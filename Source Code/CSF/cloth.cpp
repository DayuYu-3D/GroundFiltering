#include "cloth.h"

#include <omp.h>
#include <queue>
#include <fstream>
#include<iostream>

Cloth::Cloth(const Vec3d &originPos,
             int numWidth_particles,
             int numLength_particles,
             double stepX,
             double stepY,
             double smoothThreshold,
             double altitudeThreshold,
             int rigidness,
             double timeStep,
             bool bcorrectAbnormal,
             int angle2):
    _constraintIterations(rigidness),
    _timeStep(timeStep),
    _smoothThreshold(smoothThreshold),
    _altitudeThreshold(altitudeThreshold),
    _originPos(originPos),
    _stepX(stepX),
    _stepY(stepY),
    _numWdith_particles(numWidth_particles),
    _numLength_particles(numLength_particles),
    _bcorrectAbnomal(bcorrectAbnormal),
    _angle2(angle2)
{
    //预留空间,防止内存翻倍增长
    _vParticles.resize(_numLength_particles * _numWdith_particles);

    double time_step2 = _timeStep * _timeStep;

    // creating particles in a grid of particles from (0,0,0) to (width,-height,0)
    for (int i = 0; i < _numLength_particles; ++i) {
        for (int j = 0; j < _numWdith_particles; ++j) {
            Vec3d pos(_originPos[0] + j * _stepX,
                      _originPos[1] + i * _stepY,
                      _originPos[2]);

            // insert particle in column i at j'th row
            Particle pt(pos, time_step2);
            pt._posI = i;
            pt._posJ = j;
            _vParticles[i * _numWdith_particles + j] = pt;
        }
    }

    // Connecting immediate neighbor particles with constraints
    // (distance 1 and sqrt(2) in the grid)
    for (int i = 0; i < _numLength_particles; ++i) {
        for (int j = 0; j < _numWdith_particles; ++j) {

            if (j < _numWdith_particles - 1) {
                makeConstraint(getParticle(i, j), getParticle(i, j + 1));
            }

            if (i < _numLength_particles - 1) {
                makeConstraint(getParticle(i, j), getParticle(i + 1, j));
            }

            if ((i < _numLength_particles - 1) && (j < _numWdith_particles - 1)) {
                makeConstraint(getParticle(i, j), getParticle(i + 1, j + 1));
                makeConstraint(getParticle(i, j + 1), getParticle(i + 1, j));
            }
        }
    }

    // Connecting secondary neighbors with constraints (distance 2 and sqrt(4) in the grid)
    for (int i = 0; i < _numLength_particles; ++i) {
        for (int j = 0; j < _numWdith_particles; ++j) {

            if (j < _numWdith_particles - 2) {
                makeConstraint(getParticle(i, j), getParticle(i, j + 2));
            }

            if (i < _numLength_particles - 2) {
                makeConstraint(getParticle(i, j), getParticle(i + 2, j));
            }

            if ((i < _numLength_particles - 2) && (j < _numWdith_particles - 2)) {
                makeConstraint(getParticle(i, j), getParticle(i + 2, j + 2));
                makeConstraint(getParticle(i, j + 2), getParticle(i + 2, j));
            }
        }
    }
}

double Cloth::timeStep()
{
    int particleCount = static_cast<int>(_vParticles.size()); //omp不支持size_t
    #pragma omp parallel for
    for (int i = 0; i < particleCount; ++i) {
        _vParticles[i].timeStep(); //更新外力
    }

//    #pragma omp parallel for
    for (int i = 0; i < _numLength_particles; ++i) {//更新内部力
        for (int j = 0; j < _numWdith_particles; ++j) {
            getParticle(i, j)->satisfyConstraintSelf(_constraintIterations);
        }
    }

    double maxDiff = 0;
    for (int i = 0; i < particleCount; ++i) {
        if (_vParticles[i].isMovable()) {
            double diff = fabs(_vParticles[i]._oldPos[2] - _vParticles[i]._pos[2]);

            if (diff > maxDiff) {
                maxDiff = diff;
            }
        }
    }
    return maxDiff;
}

void Cloth::addForce(const Vec3d direction)
{
    int numP = static_cast<int>(_vParticles.size());
    #pragma omp parallel for
    for (int i = 0; i < numP; ++i) { //耗时较少,但是有除法运算
        _vParticles[i].addForce(direction);
    }
}

void Cloth::terrCollision()
{
    int particleCount = static_cast<int>(_vParticles.size());
    #pragma omp parallel for
    for (int i = 0; i < particleCount; ++i) {
        Vec3d v = _vParticles[i].getPos();

        if (v[2] < _vHeightvals[i]) {
            _vParticles[i].offsetPos(Vec3d(0, 0, _vHeightvals[i] - v[2]));
            _vParticles[i].setunMovable();
        }
    }
}

void Cloth::movableFilter()
{
    std::vector<Particle> tmpParticles;
    for (int i = 0; i < _numLength_particles; ++i) {
        for (int j = 0; j < _numWdith_particles; ++j) {
            Particle *ptc = getParticle(i, j);

            if (ptc->isMovable() && !ptc->_isVisited) {
                std::queue<int> que;
                std::vector<ij> connected; // store the connected component
                std::vector<std::vector<int> > neibors;
                int sum   = 1;
                int index = i * _numWdith_particles + j;

                // visit the init node
                connected.push_back(ij(i, j));
                _vParticles[index]._isVisited = true;

                // enqueue the init node
                que.push(index);

                //当队列不为空
                while (!que.empty()) {
                    Particle *ptc_f = &_vParticles[que.front()];
                    que.pop();
                    int cur_i = ptc_f->_posI;
                    int cur_j = ptc_f->_posJ;
                    std::vector<int> neibor;

                    if (cur_j > 0) {
                        ///左边的粒子
                        Particle *ptc_left = getParticle(cur_i, cur_j - 1);

                        if (ptc_left->isMovable()) {
                            if (!ptc_left->_isVisited) {
                                sum++;
                                ptc_left->_isVisited = true;
                                connected.push_back(ij(cur_i, cur_j - 1));
                                que.push(_numWdith_particles * cur_i + cur_j - 1);
                                neibor.push_back(sum - 1);
                                ptc_left->_cPos = sum - 1;
                            } else {
                                neibor.push_back(ptc_left->_cPos);
                            }
                        }
                    }

                    if (cur_j < _numWdith_particles - 1) {
                        Particle *ptc_right = getParticle(cur_i, cur_j + 1);

                        if (ptc_right->isMovable()) {
                            if (!ptc_right->_isVisited) {
                                sum++;
                                ptc_right->_isVisited = true;
                                connected.push_back(ij(cur_i, cur_j + 1));
                                que.push(_numWdith_particles * cur_i + cur_j + 1);
                                neibor.push_back(sum - 1);
                                ptc_right->_cPos = sum - 1;
                            } else {
                                neibor.push_back(ptc_right->_cPos);
                            }
                        }
                    }

                    if (cur_i > 0) {
                        Particle *ptc_bottom = getParticle(cur_i - 1, cur_j);

                        if (ptc_bottom->isMovable()) {
                            if (!ptc_bottom->_isVisited) {
                                sum++;
                                ptc_bottom->_isVisited = true;
                                connected.push_back(ij(cur_i - 1, cur_j));
                                que.push(_numWdith_particles * (cur_i - 1) + cur_j);
                                neibor.push_back(sum - 1);
                                ptc_bottom->_cPos = sum - 1;
                            } else {
                                neibor.push_back(ptc_bottom->_cPos);
                            }
                        }
                    }

                    if (cur_i < _numLength_particles - 1) {
                        Particle *ptc_top = getParticle(cur_i + 1, cur_j);

                        if (ptc_top->isMovable()) {
                            if (!ptc_top->_isVisited) {
                                sum++;
                                ptc_top->_isVisited = true;
                                connected.push_back(ij(cur_i + 1, cur_j));
                                que.push(_numWdith_particles * (cur_i + 1) + cur_j);
                                neibor.push_back(sum - 1);
                                ptc_top->_cPos = sum - 1;
                            } else {
                                neibor.push_back(ptc_top->_cPos);
                            }
                        }
                    }
                    neibors.push_back(neibor);
                }

                if (sum > MAX_PARTICLE_FOR_POSTPROCESSIN) {
                    std::vector<int> edgePoints = findUnmovablePoint(connected);
                    handleSlopConnected(edgePoints, connected, neibors);
                }
            }
        }
    }
}

std::vector<int> Cloth::findUnmovablePoint(std::vector<ij> connected)
{
    std::vector<int> edgePoints;

    std::size_t numVConnect = connected.size();
    for (std::size_t i = 0; i < numVConnect; ++i) {
        int i_ij = connected[i].i;
        int j_ij = connected[i].j;
        int index     = i_ij * _numWdith_particles + j_ij;
        Particle *ptc = getParticle(i_ij, j_ij);

        if (j_ij > 0) {
            Particle *ptc_x = getParticle(i_ij, j_ij - 1);

            if (!ptc_x->isMovable()) {
                int index_ref = i_ij * _numWdith_particles + j_ij - 1;

                if ((fabs(_vHeightvals[index] - _vHeightvals[index_ref]) < _smoothThreshold) &&
                        (ptc->getPos()[2] - _vHeightvals[index] < _altitudeThreshold)) {
                    Vec3d offsetVec = Vec3d(0, 0, _vHeightvals[index] - ptc->getPos()[2]);
                    _vParticles[index].offsetPos(offsetVec);
                    ptc->setunMovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (j_ij < j_ij - 1) {
            Particle *ptc_x = getParticle(i_ij, j_ij + 1);

            if (!ptc_x->isMovable()) {
                int index_ref = i_ij * _numWdith_particles + j_ij + 1;

                if ((fabs(_vHeightvals[index] - _vHeightvals[index_ref]) < _smoothThreshold) &&
                        (ptc->getPos()[2] - _vHeightvals[index] < _altitudeThreshold)) {
                    Vec3d offsetVec = Vec3d(0, 0, _vHeightvals[index] - ptc->getPos()[2]);
                    _vParticles[index].offsetPos(offsetVec);
                    ptc->setunMovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (i_ij > 0) {
            Particle *ptc_y = getParticle(i_ij - 1, j_ij);

            if (!ptc_y->isMovable()) {
                int index_ref = (i_ij - 1) * _numWdith_particles + j_ij;

                if ((fabs(_vHeightvals[index] - _vHeightvals[index_ref]) < _smoothThreshold) &&
                        (ptc->getPos()[2] - _vHeightvals[index] < _altitudeThreshold)) {
                    Vec3d offsetVec = Vec3d(0, 0, _vHeightvals[index] - ptc->getPos()[2]);
                    _vParticles[index].offsetPos(offsetVec);
                    ptc->setunMovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (i_ij < _numLength_particles - 1) {
            Particle *ptc_y = getParticle(i_ij + 1, j_ij);

            if (!ptc_y->isMovable()) {
                int index_ref = (i_ij + 1) * _numWdith_particles + j_ij;

                if ((fabs(_vHeightvals[index] - _vHeightvals[index_ref]) < _smoothThreshold) &&
                        (ptc->getPos()[2] - _vHeightvals[index] < _altitudeThreshold)) {
                    Vec3d offsetVec = Vec3d(0, 0, _vHeightvals[index] - ptc->getPos()[2]);
                    _vParticles[index].offsetPos(offsetVec);
                    ptc->setunMovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }
    }

    return edgePoints;
}

void Cloth::handleSlopConnected(std::vector<int> edgePoints,
                                std::vector<ij> connected,
                                std::vector<std::vector<int> > neibors)
{
    std::vector<bool> visited;

    for (std::size_t i = 0; i < connected.size(); ++i) {
        visited.push_back(false);
    }

    std::queue<int> que;

    for (std::size_t i = 0; i < edgePoints.size(); ++i) {
        que.push(edgePoints[i]);
        visited[edgePoints[i]] = true;
    }

    while (!que.empty()) {
        int index = que.front();
        que.pop();

        int index_center = connected[index].i * _numWdith_particles + connected[index].j;

        for (std::size_t i = 0; i < neibors[index].size(); ++i) {
            int index_neibor = connected[neibors[index][i]].i * _numWdith_particles + connected[neibors[index][i]].j;

            if ((fabs(_vHeightvals[index_center] - _vHeightvals[index_neibor]) < _smoothThreshold) &&
                    (fabs(_vParticles[index_neibor].getPos()[2] - _vHeightvals[index_neibor]) < _altitudeThreshold)) {
                Vec3d offsetVec = Vec3d(0, 0, _vHeightvals[index_neibor] - _vParticles[index_neibor].getPos()[2]);
                _vParticles[index_neibor].offsetPos(offsetVec);
                _vParticles[index_neibor].setunMovable();

                if (visited[neibors[index][i]] == false) {
                    que.push(neibors[index][i]);
                    visited[neibors[index][i]] = true;
                }
            }
        }
    }
}

void Cloth::saveToFile(std::string path)
{
    std::string filepath;

    if (path == "") {
        filepath = "../cloth_nodes.txt";
    } else {
        filepath = path;
    }

    std::ofstream f1(filepath.c_str());

    if (!f1) {
        return;
    }

    for (std::size_t i = 0; i < _vParticles.size(); ++i) {
        f1 << std::fixed  << _vParticles[i].getPos()[0] << " "
           << _vParticles[i].getPos()[1] << " " << -_vParticles[i].getPos()[2] << std::endl;
    }

    f1.close();
}

//TODO: 效率低待修改
void Cloth::saveMovableToFile(std::string path)
{
    std::string filepath;

    if (path == "") {
        filepath = "../cloth_movable.txt";
    } else {
        filepath = path;
    }

    std::ofstream f1(filepath.c_str());

    if (!f1) {
        return;
    }

    for (std::size_t i = 0; i < _vParticles.size(); i++) {
        if (_vParticles[i].isMovable()) {
            f1 << std::fixed  << _vParticles[i].getPos()[0] << " "
               << _vParticles[i].getPos()[1] << " " << -_vParticles[i].getPos()[2] << std::endl;
        }
    }

    f1.close();
}

void Cloth::makeConstraint(Particle *p1, Particle *p2)
{
    p1->_neighborsList.push_back(p2);
    p2->_neighborsList.push_back(p1);
}

Particle *Cloth::getParticle(int i, int j)
{
    return &_vParticles[i * _numWdith_particles + j];
}

