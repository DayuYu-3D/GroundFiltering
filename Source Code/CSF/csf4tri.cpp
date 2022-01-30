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
    std::cout << "Configuring terrain..." << std::endl;
    Vec3d bbMin, bbMax;
    this->computeBoundingBox(bbMin, bbMax);
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

    std::cout << "Finding intersection height..." << std::endl;
    ParticalAltitudeFinder partiAF(cloth, _pVTri, _mvVertex);
    partiAF.findIntersectionHeight();

    double timeStep2 = _params.timeStep * _params.timeStep;
    std::cout << "Find couple triangle done" << std::endl;

    auto startT = std::chrono::system_clock::now(); //获取开始时间
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
    auto entT = std::chrono::system_clock::now();//获取结束时间
    double ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "Simulat done, use time: " << (double)ut / 1000 << "s" << std::endl;
    std::cout << "Number of Iteraion: " << ni << "; Max distance: " << maxDiff << std::endl;

    if (_params.bSloopSmooth) {
        startT = std::chrono::system_clock::now();
        std::cout << "--- post handle..." << std::endl;
        cloth.movableFilter();
        entT = std::chrono::system_clock::now();//获取结束时间
        ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
        std::cout << "--- post handle done, use time:" << (double)ut / 1000 << "s" << std::endl;
    }

    if (1) {
        cloth.saveToFile();
    }
    if (1)
    {
        cloth.saveMovableToFile();
    }

    SplitTriangles2Terrain splitTriangle(_params.classThreshold);
    splitTriangle.splitTerrain(cloth, _pVTri, _mvVertex, groundIndexs, offGroundIndexs);

    std::cout << "Split done, Nums of ground triangles:" << groundIndexs.size()
              << "Nums of off ground triangles: " << offGroundIndexs.size() << std::endl;
}
void CSF4Tri::computeBoundingBox(Vec3d &bbmin, Vec3d &bbmax) const
{
    bbmin = Vec3d(FLT_MAX, FLT_MAX, FLT_MAX), bbmax = Vec3d(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    std::vector<int> ix;
    for(auto& geom :*_mvVertex)
        ix.emplace_back(geom.first);

    size_t n1 = 0, n2 = 0;
    n1 = ix.size();
    for(size_t i = 0; i < n1; ++i)
    {
        const size_t& geomIx = ix[i];
        n2 = _mvVertex->at(geomIx).size();
        for(size_t j = 0; j < n2; ++j)
            for (int d = 0; d < 3; ++d)
            {
                const Vertex& vex = _mvVertex->at(geomIx).at(j);
                if (vex._coor[d] < bbmin[d]) {
                    bbmin[d] = vex._coor[d];
                }
                else if (vex._coor[d] > bbmax[d]) {
                    bbmax[d] = vex._coor[d];
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
