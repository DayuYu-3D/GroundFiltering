#include "csf4tri.h"

#include <iostream>
#include <algorithm>
#include <chrono>

#include "vec3d.h"
#include "cloth.h"
#include "particalaltitudefinder.h"
#include "splittriangles2terrain.h"

CSF4Tri::CSF4Tri()
    : _pVTri(nullptr),
      _mvVertex(nullptr)
{
}

CSF4Tri::~CSF4Tri()
{

}

void CSF4Tri::setParams(const Params ps)
{
    this->_params = ps;
}

void CSF4Tri::doCSFFilter(std::vector<unsigned int> &groundIndexs,
                          std::vector<unsigned int> &offGroundIndexs) const
{
    auto startT = std::chrono::system_clock::now(); //获取开始时间

    std::cout << "Configuring terrain..." << std::endl;
    Vec3d bbMin, bbMax;
    this->computeBoundingBox(bbMin, bbMax);
    std::cout << "Configuring terrain..." << std::endl;
    std::cout << "Min value:" << bbMin[0] << ", " << bbMin[1] << ", " << bbMin[2] << std::endl;
    std::cout << "Max value:" << bbMax[0] << ", " << bbMax[1] << ", " << bbMax[2] << std::endl;

    double clothHeight_z = 0.05;
    int clothBuffer = 2;//使布料比mesh大一点
    double gravity = 0.2;

    Vec3d originPos;
    originPos.x() = bbMin.x() - clothBuffer * _params.clothResolution;
    originPos.y() = bbMin.y() - clothBuffer * _params.clothResolution;
    originPos.z() = bbMax.z() + clothHeight_z;

    int numWidth = static_cast<int>(std::floor((bbMax[0] - bbMin[0]) / _params.clothResolution)) + 2 * clothBuffer;
    int numlength = static_cast<int>(std::floor((bbMax[1] - bbMin[1]) / _params.clothResolution)) + 2 * clothBuffer;


    std::cout << "Configuring cloth..." << std::endl;
    std::cout << "Width of cloth:" << numWidth << std::endl;
    std::cout << "Length of cloth:" << numlength << std::endl;

    //tan15°=0.26794
    Cloth cloth(originPos, numWidth, numlength,
                _params.clothResolution, _params.clothResolution,
                0.26794 * _params.clothResolution, 9999,
                _params.rigidness, _params.timeStep, _params.bCorrectAbnormal, _params.angle2);
    std::cout << "Configuring terrain & cloth done" << std::endl;
    auto entT = std::chrono::system_clock::now();//获取结束时间
    double ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout <<"   ** use time: " << (double)ut / 1000 << "s" << std::endl;

    startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "Finding intersection height..." << std::endl;
    ParticalAltitudeFinder partiAF(cloth, _pVTri, _mvVertex);
    partiAF.findIntersectionHeight();

    double timeStep2 = _params.timeStep * _params.timeStep;
    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "Find couple triangle done\n";
    std::cout <<"   ** use time: " << (double)ut / 1000 << "s" << std::endl;

    startT = std::chrono::system_clock::now(); //获取开始时间
    std::cout << "Simulating..." << std::endl;
    cloth.addForce(Vec3d(0, 0, -gravity) * timeStep2); //施加重力
    int ni = 0;
    double maxDiff = 0;
    for (; ni < _params.interations; ++ni) {
        maxDiff = cloth.timeStep(); //更新位移
        cloth.terrCollision(); //地形碰撞检测
        if ((maxDiff != 0) && (maxDiff < 0.005)) { //所有粒子的上一次迭代位移与这一次的距离，的最大值
            break;
        }
    }
    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "Simulat done\n";
    std::cout << "   ** use time: " << (double)ut / 1000 << "s" << std::endl;
    std::cout << "Number of Iteraion: " << ni << "; Max distance: " << maxDiff << std::endl;

    if (_params.bSloopSmooth) {
        startT = std::chrono::system_clock::now();
        std::cout << "--- post handle..." << std::endl;
        cloth.movableFilter();
        entT = std::chrono::system_clock::now();//获取结束时间
        ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
        std::cout << "--- post handle done, use time:" << (double)ut / 1000 << "s" << std::endl;
    }

    if (0) {
        cloth.saveToFile();
    }
    if (0)
    {
        cloth.saveMovableToFile();
    }

    startT = std::chrono::system_clock::now(); //获取开始时间
    SplitTriangles2Terrain splitTriangle(_params.classThreshold);
    splitTriangle.splitTerrain(cloth, _pVTri, _mvVertex, groundIndexs, offGroundIndexs);
    entT = std::chrono::system_clock::now();//获取结束时间
    ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "Split done, Nums of ground triangles:" << groundIndexs.size()
              << "Nums of off ground triangles: " << offGroundIndexs.size() << std::endl;
    std::cout << "   ** use time: " << (double)ut / 1000 << "s" << std::endl;
}
void CSF4Tri::computeBoundingBox(Vec3d &bbmin, Vec3d &bbmax) const
{
    bbmin = Vec3d(FLT_MAX, FLT_MAX, FLT_MAX), bbmax = Vec3d(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    int n1 = 0;
    n1 = _pVTri->size();
#pragma omp parallel for
    for(int i = 0; i< n1; ++i)
    {
        const int& geomIx = _pVTri->at(i)._geomIndex;
        const int& vix1 = _pVTri->at(i)._vertexIndexs[0];
        const int& vix2 = _pVTri->at(i)._vertexIndexs[1];
        const int& vix3 = _pVTri->at(i)._vertexIndexs[2];

        const Vertex& vex1 = _mvVertex->at(geomIx).at(vix1);
        const Vertex& vex2 = _mvVertex->at(geomIx).at(vix2);
        const Vertex& vex3 = _mvVertex->at(geomIx).at(vix3);
        for (int d = 0; d < 3; ++d)
        {
            if (vex1._coor[d] < bbmin[d]) {
#pragma omp critical
                bbmin[d] = vex1._coor[d];
            }
            else if (vex1._coor[d] > bbmax[d]) {
#pragma omp critical
                bbmax[d] = vex1._coor[d];
            }

            if (vex2._coor[d] < bbmin[d]) {
#pragma omp critical
                bbmin[d] = vex2._coor[d];
            }
            else if (vex2._coor[d] > bbmax[d]) {
#pragma omp critical
                bbmax[d] = vex2._coor[d];
            }

            if (vex3._coor[d] < bbmin[d]) {
#pragma omp critical
                bbmin[d] = vex3._coor[d];
            }
            else if (vex3._coor[d] > bbmax[d]) {
#pragma omp critical
                bbmax[d] = vex3._coor[d];
            }
        }

    }
}

void CSF4Tri::setTriangleCloud(const std::vector<Triangle> *vTri,
                               const std::map<size_t, std::vector<Vertex> > *vvVertex)
{
    _pVTri = vTri;
    _mvVertex = vvVertex;
}
