#include "particalaltitudefinder.h"

#include <omp.h>

#include <math.h>
#include <iostream>
#include <chrono>
#include <queue>
#include <map>
#include<numeric>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#define PI acos(-1)

namespace  {
inline double dist22(double x1, double y1, double x2, double y2)
{
    return ((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2));
}

inline double sign11(double x1, double y1, double x2, double y2,
                     double x3, double y3)
{
    return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
}

/**
 * @brief pointInTriangle2D:判断一个二维点是否在三角形内部
 * @param point2D: 二维点
 * @param v1: 第一个顶点
 * @param v2: 第二个顶点
 * @param v3: 第三个顶点
 * @return 在则true,否则false
 */
inline bool pointInTriangle2D(
    const double *point2D, const Vec3d &v1,
    const Vec3d &v2, const Vec3d &v3)
{
    if(((v1[0] == v2[0]) && (v1[1] == v2[1])) ||
            ((v2[0] == v3[0]) && (v2[1] == v3[1])) ||
            ((v1[0] == v3[0]) && (v1[1] == v3[1])))
    {
        return false;//如果存在两点相等,退出
    }

    double d1 = sign11(point2D[0], point2D[1], v1[0], v1[1], v2[0], v2[1]);
    double d2 = sign11(point2D[0], point2D[1], v2[0], v2[1], v3[0], v3[1]);
    double d3 = sign11(point2D[0], point2D[1], v3[0], v3[1], v1[0], v1[1]);

    return !(((d1 < 0) || (d2 < 0) || (d3 < 0)) && ((d1 > 0) || (d2 > 0) || (d3 > 0)));
}

}

/***************************************************************************************/
/***************************************************************************************/
/***************************************************************************************/
/***************************************************************************************/

ParticalAltitudeFinder::ParticalAltitudeFinder(Cloth &cloth,
        const std::vector<Triangle> *pvTriangle,
        const std::map<size_t, std::vector<Vertex> > *pvvVertex):
    _cloth(cloth),
    _pvTriangle(pvTriangle),
    _pmvVertex(pvvVertex),
    _pBVH(nullptr),
    _pTraverser(nullptr)
{

}

ParticalAltitudeFinder::~ParticalAltitudeFinder()
{
    if(_pBVH)
    {
        delete  _pBVH;
    }
    if(_pTraverser)
    {
        delete  _pTraverser;
    }
}

void ParticalAltitudeFinder::findIntersectionHeight()
{
    //1. 循环所有顶点,将其分配到cloth的粒子网格内
    auto startT = std::chrono::system_clock::now();///获取当前时间
    this->findCollsionHeight_bvh();
    auto afterT = std::chrono::system_clock::now();///获取结束时间
    std::cout << "## Find couple triangle use time: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(afterT - startT).count() << "ms" << std::endl;

    //4.查看是否还有空高程,若有则寻找最近粒子点的高程
    auto& heightVal = _cloth.getHeightvals();
    int numNullPt = 0;
    for(size_t i=0; i<heightVal.size(); ++i) {
        if(heightVal[i] == MIN_INF) {
            numNullPt += 1;
            auto* pt = _cloth.getParticle1d(i);
            heightVal[i] = findHeightValByScanline(pt, _cloth);//为空值搜索高程值
        }
    }
    int numNullPt1 = 0;
    for(size_t i=0; i<heightVal.size(); ++i) {
        if(heightVal[i] == MIN_INF) {
            numNullPt1 += 1;
        }
    }
    std::cout << "## Total num of partical:" << _cloth.getSize() <<"; Number of particals not find couple tirangle: "
              << numNullPt <<"; After find neighbor: "<<numNullPt1<<std::endl;

    this->saveParticlesAndHeight();

    //进行渐进地形异常高程值检测及恢复
    if(_cloth._bcorrectAbnomal)
    {
        std::cout<<"no abnormal particles correction"<<std::endl;
       this->findAbnormalParticles();
    }else
        std::cout<<"abnormal particles correction"<<std::endl;
}

void ParticalAltitudeFinder::findCollsionHeight_nearstVertex()
{
    //1. 循环所有顶点,将其分配到cloth的粒子网格内
    std::vector<int> ix;
    for(auto& geom :*_pmvVertex)
    {
        ix.emplace_back(geom.first);
    }
    size_t nGeom = ix.size();
    std::cout<< "sssss: "<<nGeom<<std::endl;
    for(size_t i = 0; i < nGeom; ++i) //不耗时
    {
        size_t geomIx = ix[i];
        size_t npt = _pmvVertex->at(geomIx).size();
        for(size_t j = 0; j < npt; ++j)
        {
            Vertex point = _pmvVertex->at(geomIx)[j];
            double pointX = point._coor[0];
            double pointY = point._coor[1];

            double deltaX = pointX - _cloth._originPos[0];
            double deltaY = pointY - _cloth._originPos[1];
            int    col    = int(deltaX / _cloth._stepX + 0.5);
            int    row    = int(deltaY / _cloth._stepY + 0.5);

            if ((col >= 0) && (row >= 0)) {
                Particle *pt = _cloth.getParticle(row, col);
                pt->_correspondingLidarPointList.push_back(i);//可删除,同_nearestPointIndex
                double dist = dist22(
                                  pointX, pointY,
                                  pt->getPos()[0],
                                  pt->getPos()[1]
                              );

                if (dist < pt->_tmpDist) {
                    pt->_tmpDist           = dist;
                    pt->_nearestPointHeight = point._coor[2];
                    pt->_nearestPointIndex  = i; //最邻近点所在的三角形索引
                }
            }
        }
    }

    ////不知道这段代码有什么用，应该是我当时写好后，有更好的方案，被舍弃了？
    //2. 为没有找到CT的粒子寻找CT,分为两种:一种是本身就是没有CT点的粒子,对应位置没有三角网;
    //一种是由于部分三角网较大,粒子网格范围内没有顶点
//    int numNullPt = 0;
//    auto nt = pvTriangle->size();
////    #pragma omp parallel for
//    for (int i = 0; i < cloth.getSize(); ++i) {
//        Particle *pcur = cloth.getParticle1d(i);
//        double nearestHeight = pcur->_nearestPointHeight;
//        if (nearestHeight > MIN_INF) {

////            double point[2] = {pcur->_pos[0], pcur->_pos[1]};
////            for(size_t t = 0; t < nt; ++t) {
////                auto gemIx = pvTriangle->at(t)._geomIndex;
////                Vec3d v1 = pvvVertex->at(gemIx).at(pvTriangle->at(t)._vertexIndexs[0])._coor;
////                Vec3d v2 = pvvVertex->at(gemIx).at(pvTriangle->at(t)._vertexIndexs[1])._coor;
////                Vec3d v3 = pvvVertex->at(gemIx).at(pvTriangle->at(t)._vertexIndexs[2])._coor;
////                if(pointInTriangle2D(point, v1, v2, v3)) {
////                    numNullPt += 1;
//////                    std::cout << "tri  " << point[0] << " " << point[1] << std::endl;
//////                    std::cout << v1[0] << " " << v1[1] << std::endl;
//////                    std::cout << v2[0] << " " << v2[1] << std::endl;
//////                    std::cout << v3[0] << " " << v3[1] << std::endl;
////                    break;
////                }
////            }
//            numNullPt += 1;
//        }
//    }
//    std::cout << "Total num of partical:" << cloth.getSize() <<
//              "; Number of particals not find couple tirangle:" << numNullPt  << std::endl;

    //3.高程数组赋值
    //将最近点的顶点的高程值赋值给粒子的CT高程值
    std::vector<double> &heightVal = _cloth.getHeightvals();
    heightVal.reserve(_cloth.getSize() + 1);
    heightVal.resize(_cloth.getSize());
    auto numParticles = _cloth.getSize();
    for (int i = 0; i < numParticles; ++i) { //不耗时
        Particle *pcur = _cloth.getParticle1d(i);
        double CTHeight = pcur->_nearestPointHeight;
        if (CTHeight > MIN_INF) {
            heightVal[i] = CTHeight;
        } else {
            heightVal[i] = MIN_INF;//为空值搜索高程值
        }
    }
}

void ParticalAltitudeFinder::findCollsionHeight_bvh()
{
    std::vector<std::uint32_t> face_indices;
    std::size_t nFaces = _pvTriangle->size();
    face_indices.resize(nFaces);
    for (std::size_t i = 0; i < nFaces; ++i) {
      face_indices[i] = std::uint32_t(i);
    }

    //构建BVH树，速度有点慢
    if(_pBVH)
    {
        delete _pBVH;
        _pBVH = nullptr;
    }
    auto startT = std::chrono::system_clock::now();///获取当前时间
    std::printf("## Building BVH\n");
    FaceBoxConverter<float> box_converter(*_pmvVertex, *_pvTriangle);
    FastBVH::BuildStrategy<float, 1> build_strategy;
    _pBVH = build_strategy(face_indices, box_converter);
    auto afterT = std::chrono::system_clock::now();///获取结束时间
    std::cout << "## Build done, consumed time: " <<
              std::chrono::duration_cast<std::chrono::milliseconds>(afterT - startT).count() << "ms" << std::endl;

    FaceIntersector<float> intersector(*_pmvVertex, *_pvTriangle); //相交判断

    _pTraverser = new FastBVH::Traverser<float, uint32_t, decltype(intersector)> (*_pBVH, intersector);

    //遍历所有粒子，获取交点
    std::vector<double> &heightVal = _cloth.getHeightvals();
    heightVal.clear();
    heightVal.resize(_cloth.getSize());
    int nParticles = _cloth.getSize();
    for(int i = 0; i<nParticles; ++i)
    {
        Particle* pt = _cloth.getParticle1d(i);
        const Vec3d& pos = pt->getPos();
        FastBVH::Vector3<float> o; //射线的起点
        o.x=pos[0];o.y=pos[1];o.z=pos[2];
        FastBVH::Vector3<float> d; //射线的方向
        d.x=0;d.y=0;d.z=-1; //竖直向下

        std::vector<FastBVH::Intersection<float, std::uint32_t>> vIntersection;
        _pTraverser->traverse(FastBVH::Ray<float>{o,d}, vIntersection);
        if(vIntersection.size())
        {
            auto isect = vIntersection[0];
            if(isect){
                auto interSection = isect.getHitPosition(o,d);
                if (interSection.z > MIN_INF) {
                    heightVal[i] = interSection.z;
                    pt->_nearestPointHeight = interSection.z;
                } else {
                      heightVal[i] = MIN_INF;
                }
            }else {
                heightVal[i] = MIN_INF;
            }
        }else {
            heightVal[i] = MIN_INF;
        }
    }
}

double ParticalAltitudeFinder::findHeightValByNeighbor(Particle *p)
{
    std::queue<Particle *>  nqueue;
    std::vector<Particle *> pbacklist;
    int neiborsize = p->_neighborsList.size();

    for (int i = 0; i < neiborsize; i++) {
        p->_isVisited = true;
        nqueue.push(p->_neighborsList[i]);
    }

    // iterate over the nqueue
    while (!nqueue.empty()) {
        Particle *pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);

        if (pneighbor->_nearestPointHeight > MIN_INF) {
            for (std::size_t i = 0; i < pbacklist.size(); i++) {
                pbacklist[i]->_isVisited = false;
            }

            while (!nqueue.empty()) {
                Particle *pp = nqueue.front();
                pp->_isVisited = false;
                nqueue.pop();
            }

            return pneighbor->_nearestPointHeight;
        } else {
            int nsize = pneighbor->_neighborsList.size();

            for (int i = 0; i < nsize; i++) {
                Particle *ptmp = pneighbor->_neighborsList[i];

                if (!ptmp->_isVisited) {
                    ptmp->_isVisited = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }

    return MIN_INF;
}

double ParticalAltitudeFinder::findHeightValByScanline( Particle *p, Cloth &cloth)
{
    //step1:先扫描线搜寻
    int ipos = p->_posI;
    int jpos = p->_posJ;

    //循环列
    for (int i = jpos + 1; i < cloth._numWdith_particles; ++i) {
        double crresHeight = cloth.getParticle(ipos, i)->_nearestPointHeight;

        if (crresHeight > MIN_INF) {
            return crresHeight;
        }
    }

    //循环列
    for (int i = jpos - 1; i >= 0; --i) {
        double crresHeight = cloth.getParticle(ipos, i)->_nearestPointHeight;

        if (crresHeight > MIN_INF) {
            return crresHeight;
        }
    }

    //循环行
    for (int j = ipos - 1; j >= 0; --j) {
        double crresHeight = cloth.getParticle(j, jpos)->_nearestPointHeight;

        if (crresHeight > MIN_INF) {
            return crresHeight;
        }
    }

    //循环行
    for (int j = ipos + 1; j < cloth._numLength_particles; ++j) {
        double crresHeight = cloth.getParticle(j, jpos)->_nearestPointHeight;

        if (crresHeight > MIN_INF) {
            return crresHeight;
        }
    }

    //如果周围邻居有高程,则使用邻居的高程值
    return findHeightValByNeighbor(p);
}

void ParticalAltitudeFinder::findAbnormalParticles()
{

    double threshold = _cloth._stepY * tan(_cloth._angle2 * std::PI/180.0f);
    double threshold_down = threshold; //下降阈值
    std::cout<<threshold<<std::endl;
    double threshold2 = threshold; //假异常前后的梯度变化阈值，单位米,
    auto& heightVal = _cloth.getHeightvals();

    //5.异常粒子高程值处理
    //异常粒子是指地面底下的噪音面片
    std::map<int,std::vector<double>> mAbnormalParticles; //int为异常粒子的索引，vector为异常粒子的前一正常点的高程值（上下左右个一个）
    int nWdith =  _cloth._numWdith_particles; //列数
    int nLength = _cloth._numLength_particles;//行数
    double lastHeight = 0; //上一个有效粒子点的高程值

    //按行 从左到右遍历
    for(int row = 0; row < nLength; ++row)
    {
        //取该行的平均值
        double meanHeight = 0;
        for(int col = 0; col < nWdith; ++col)
        {
            int ix = row * nWdith + col;
            meanHeight += heightVal[ix];
        }
        meanHeight /= nWdith;

//        lastHeight = heightVal[row * nWdith + 0]; //deprivated
        bool isFirstOne = true;
        int counts = 0; //用于记录连续的假/夹噪音的个数
        for(int col = 0; col < nWdith; ++col)
        {
            int ix = row * nWdith + col;
            //当前粒子的高程值
            double curHeight = heightVal[ix];

            if( isFirstOne && curHeight < meanHeight)
            {//说明这个起始点有可能是建筑物点
             //，为防止后续所有点被标记为真异常点
                continue;
            }
            if(isFirstOne){ //让初始lastHeight为第一个大于meanHeight的粒子的高程值
                lastHeight = curHeight;
                isFirstOne = false;
            }

            //判断当前点与上一有效点的高程插值是否超过阈值
            double tmp = threshold2 * counts;
            if(curHeight - lastHeight - tmp > threshold) //第一种情况真异常点，即噪声点
            {
                //添加到异常点列表中，lastHeight不变

                auto it = mAbnormalParticles.find(ix);

                if(it != mAbnormalParticles.end())
                {//已经有了
                    it->second.emplace_back(lastHeight);
                }else{
                    std::vector<double> tempVec;
                    tempVec.emplace_back(lastHeight);
                    mAbnormalParticles.insert(std::make_pair(ix,tempVec));
                }

                counts += 1;
            }
            else if(lastHeight-curHeight > threshold_down)//第二种情况假异常点，可能是建筑物点
            {
                //不添加到列表中，lastHeight不变
//                counts += 1;
            }
            else
            {
                lastHeight = curHeight;
                counts = 0;
            }
        }
    }
    std::cout<<"###Abnormal particle number LEFT-RIGHT: "<<mAbnormalParticles.size()<<std::endl;
    //按行：从右到左
    for(int row = 0; row < nLength; ++row)
    {
        //取该行的平均值
        double meanHeight = 0;
        for(int col = 0; col < nWdith; ++col)
        {
            int ix = row * nWdith + col;
            meanHeight += heightVal[ix];
        }
        meanHeight /= nWdith;

//        lastHeight = heightVal[row * nWdith + nWdith - 1];//deprivated
        bool isFirstOne = true;
        int counts = 0;
        for(int col = nWdith - 1; col >=0; --col)
        {
            int ix = row * nWdith + col;
            //当前粒子的高程值
            double curHeight = heightVal[ix];

            if(isFirstOne && curHeight < meanHeight)
            {//说明这个起始点有可能是建筑物点
             //，为防止后续所有点被标记为真异常点
                continue;
            }
            if(isFirstOne){ //让初始lastHeight为第一个大于meanHeight的粒子的高程值
                lastHeight = curHeight;
                isFirstOne = false;
            }

            //判断当前点与上一有效点的高程插值是否超过阈值
            double tmp = threshold2 * counts;
            if(curHeight - lastHeight - tmp > threshold) //第一种情况真异常点，即噪声点
            {
                //添加到异常点列表中，lastHeight不变
                auto it = mAbnormalParticles.find(ix);
                if(it != mAbnormalParticles.end())
                {//已经有了
                    it->second.emplace_back(lastHeight);
                }else{
                    std::vector<double> tempVec;
                    tempVec.emplace_back(lastHeight);
                    mAbnormalParticles.insert(std::make_pair(ix,tempVec));
                }
                counts += 1;
            }
            else if(lastHeight-curHeight > threshold_down)//第二种情况假异常点，可能是建筑物点
            {
                //不添加到列表中，lastHeight不变
//                counts += 1;
            }
            else
            {
                lastHeight = curHeight;
                counts = 0;
            }
        }
    }
     std::cout<<"###Abnormal particle number: "<<mAbnormalParticles.size()<<std::endl;
    //按列 从下到上遍历
    for(int col = 0; col < nWdith; ++col)
    {
        //取该行的平均值
        double meanHeight = 0;
        for(int row = 0; row < nLength; ++row)
        {
            int ix = row * nWdith + col;
            meanHeight += heightVal[ix];
        }
        meanHeight /= nLength;

//        lastHeight = heightVal[0 * nWdith + col];//deprivated
        bool isFirstOne = true;
        int counts = 0;
        for(int row = 0; row < nLength; ++row)
        {
            int ix = row * nWdith + col;
            //当前粒子的高程值
            double curHeight = heightVal[ix];

            if(isFirstOne && curHeight < meanHeight)
            {//说明这个起始点有可能是建筑物点
             //，为防止后续所有点被标记为真异常点
                continue;
            }
            if(isFirstOne){ //让初始lastHeight为第一个大于meanHeight的粒子的高程值
                lastHeight = curHeight;
                isFirstOne = false;
            }

            //判断当前点与上一有效点的高程插值是否超过阈值
            double tmp = threshold2 * counts;
            if(curHeight - lastHeight - tmp > threshold) //第一种情况真异常点，即噪声点
            {
                //添加到异常点列表中，lastHeight不变

                auto it = mAbnormalParticles.find(ix);

                if(it != mAbnormalParticles.end())
                {//已经有了
                    it->second.emplace_back(lastHeight);
                }else{
                    std::vector<double> tempVec;
                    tempVec.emplace_back(lastHeight);
                    mAbnormalParticles.insert(std::make_pair(ix,tempVec));
                }
                counts += 1;
            }
            else if(lastHeight-curHeight > threshold_down)//第二种情况假异常点，可能是建筑物点
            {
                //不添加到列表中，lastHeight不变
//                counts += 1;
            }
            else
            {
                lastHeight = curHeight;
                counts = 0;
            }
        }
    }
     std::cout<<"###Abnormal particle number: "<<mAbnormalParticles.size()<<std::endl;
    //按列 从下到上遍历
    for(int col = 0; col < nWdith; ++col)
    {
        //取该行的平均值
        double meanHeight = 0;
        for(int row = 0; row < nLength; ++row)
        {
            int ix = row * nWdith + col;
            meanHeight += heightVal[ix];
        }
        meanHeight /= nLength;

//        lastHeight = heightVal[(nLength-1) * nWdith + col];//deprivated
        bool isFirstOne = true;
        int counts = 0;
        for(int row = nLength-1 ; row >=0; --row)
        {
            int ix = row * nWdith + col;
            //当前粒子的高程值
            double curHeight = heightVal[ix];

            if(isFirstOne && curHeight < meanHeight)
            {//说明这个起始点有可能是建筑物点
             //，为防止后续所有点被标记为真异常点
                continue;
            }
            if(isFirstOne){ //让初始lastHeight为第一个大于meanHeight的粒子的高程值
                lastHeight = curHeight;
                isFirstOne = false;
            }

            //判断当前点与上一有效点的高程插值是否超过阈值
            double tmp = threshold2 * counts;
            if(curHeight - lastHeight - tmp > threshold) //第一种情况真异常点，即噪声点
            {
                //添加到异常点列表中，lastHeight不变

                auto it = mAbnormalParticles.find(ix);

                if(it != mAbnormalParticles.end())
                {//已经有了
                    it->second.emplace_back(lastHeight);
                }else{
                    std::vector<double> tempVec;
                    tempVec.emplace_back(lastHeight);
                    mAbnormalParticles.insert(std::make_pair(ix,tempVec));
                }
                counts += 1;
            }
            else if(lastHeight-curHeight > threshold_down)//第二种情况假异常点，可能是建筑物点
            {
                //不添加到列表中，lastHeight不变
//                counts += 1;
            }
            else
            {
                lastHeight = curHeight;
                counts = 0;
            }
        }
    }
    std::cout<<"###Abnormal particle number: "<<mAbnormalParticles.size()<<std::endl;

    //被两次标记为真异常的保留
    if(0)
    {
        auto iter = mAbnormalParticles.begin();
        while(iter != mAbnormalParticles.end())
        {
            if(iter->second.size() < 2)
            {
                mAbnormalParticles.erase(iter);
            }
            iter++;
        }
    }
    std::cout<<"###Abnormal particle number (>=2): "<<mAbnormalParticles.size()<<std::endl;

    //保存检测到的异常粒子看看
    bool saveAbonomarl = false;
    if(saveAbonomarl)
    {
        std::vector<size_t> ix11;
        auto iter = mAbnormalParticles.begin();
        while(iter != mAbnormalParticles.end())
        {
            ix11.emplace_back(iter->first);
            iter++;
        }
        this->saveAbnormalParticleAndHeight(ix11);
    }

    //方案1：取平均值
    if(0)
    {
        auto iter = mAbnormalParticles.begin();
        while(iter != mAbnormalParticles.end()) {
            int ix = iter->first;
            std::vector<double>& validHeight = iter->second;
            //暂时方案：去平均值
            double meanHeight = std::accumulate(validHeight.begin(),validHeight.end(),0.0);
            meanHeight /= validHeight.size();
            heightVal[ix] = meanHeight;
            iter++;
        }
        return;
    }

   //方案2：bvh求交点，取接近有效值的那个交点
    {
        //TODO：有问题，若不重建这个,則traverse时会出bug，很神奇，目前跟踪不进去bug，只是换了个函数就不行？
        //淦
        //构建BVH树，速度有点慢，是
        std::vector<std::uint32_t> face_indices;
        std::size_t nFaces = _pvTriangle->size();
        face_indices.resize(nFaces);
        for (std::size_t i = 0; i < nFaces; ++i) {
          face_indices[i] = std::uint32_t(i);
        }
        if(_pBVH)
        {
            delete _pBVH;
            _pBVH = nullptr;
        }
        if(_pTraverser)
        {
            delete _pTraverser;
            _pTraverser = nullptr;
        }
        auto startT = std::chrono::system_clock::now();///获取当前时间
        std::printf("## Building BVH\n");
        FaceBoxConverter<float> box_converter(*_pmvVertex, *_pvTriangle);
        FastBVH::BuildStrategy<float, 1> build_strategy;
        _pBVH = build_strategy(face_indices, box_converter);
        auto afterT = std::chrono::system_clock::now();///获取结束时间
        std::cout << "## Build done, consumed time: " <<
                  std::chrono::duration_cast<std::chrono::milliseconds>(afterT - startT).count() << "ms" << std::endl;


        FaceIntersector<float> intersector(*_pmvVertex, *_pvTriangle); //相交判断
        _pTraverser = new FastBVH::Traverser<float, uint32_t, decltype(intersector)>(*_pBVH, intersector);
        //获取每个异常粒子的所有交点
        _pTraverser->setTraverserFlags(FastBVH::TraverserFlags::AllOcclusion);
        std::vector<FastBVH::Intersection<float, std::uint32_t>> vIntersection;
        auto iter = mAbnormalParticles.begin();
        std::cout<< mAbnormalParticles.size()<<std::endl;
        for(;iter!=mAbnormalParticles.end();++iter)
        {
            int ix = iter->first;
            std::vector<double>& validHeight = iter->second;
            double minHeight = *std::min_element(validHeight.begin(),validHeight.end());
            double sum = std::accumulate(validHeight.begin(),validHeight.end(),0.0);
            double meanHeight =  sum / validHeight.size();
            double height_ = minHeight;

            Particle* pt = _cloth.getParticle1d(ix);
            const Vec3d& pos = pt->getPos();
            FastBVH::Vector3<float> o; //射线的起点
            o.x=pos[0];o.y=pos[1];o.z=pos[2];
            FastBVH::Vector3<float> d; //射线的方向
            d.x=0;d.y=0;d.z=-1; //竖直向下
            vIntersection.clear();
            _pTraverser->traverse(FastBVH::Ray<float>{o,d}, vIntersection); //若不重新构建则这句话出问题
            if(vIntersection.size()>1)
            {
                float minZ = MAX_INF;
                for(auto isect : vIntersection)
                {
                    if(isect)
                    {
                        auto interSection = isect.getHitPosition(o,d);
                        if (interSection.z > MIN_INF)
                        {
//                            std::cout<<ix<<": "<<interSection.z<<std::endl;
                            //有效的交点高程值
                            if(std::fabs(interSection.z - height_) < minZ)
                            {
                                minZ = std::fabs(interSection.z - height_);
                                heightVal[ix] = interSection.z;
                                pt->_nearestPointHeight = interSection.z;

                            }
                        } else {
                            std::cout<<"zzzzzzz\n";
                        }
                    }else {
                        std::cout<<"ssssssss\n";
                    }
                }
//                std::cout<<minHeight<<" Fianl: "<<heightVal[ix]<<std::endl;
//                for(auto it = validHeight.begin();it!=validHeight.end();++it)
//                {
//                    std::cout<<"last: "<<*it<<std::endl;
//                }
//                for(auto isect : vIntersection)
//                {
//                     auto interSection = isect.getHitPosition(o,d);
//                    std::cout<<"inter: "<<interSection.z<<std::endl;
//                }
            }
            else if (vIntersection.size() ==1 )
            {
                if(heightVal[ix] - height_ > 2)
                    heightVal[ix] = height_;
            }
            else if (vIntersection.size() ==0 )
            {
               heightVal[ix] = height_;
            }
//            std::cout<<meanHeight<<" Fianl: "<<heightVal[ix]<<std::endl;
        }
    }

    //保存检测到的异常粒子看看
    if(!saveAbonomarl)
    {
        std::vector<size_t> ix11;
        auto iter = mAbnormalParticles.begin();
        while(iter != mAbnormalParticles.end())
        {
            ix11.emplace_back(iter->first);
            iter++;
        }
        this->saveAbnormalParticleAndHeight(ix11);
    }

}

void ParticalAltitudeFinder::saveParticlesAndHeight()
{
    std::string filepath("../height_particles.txt");
    std::ofstream f1(filepath.c_str());
    const auto& heightVal = _cloth.getHeightvals();
    int np = _cloth._numWdith_particles * _cloth._numLength_particles;
    for (std::size_t i = 0; i < np; ++i) {
        auto* particle = _cloth.getParticle1d(i);
        const auto& height = heightVal[i];
            f1 << std::fixed  << particle->getPos()[0] << " "
               << particle->getPos()[1]<< " " << -height << std::endl;

    }

    f1.close();
}

void ParticalAltitudeFinder::saveAbnormalParticleAndHeight(std::vector<size_t> &ixParticle)
{
    std::string filepath("../height_particles_abnormal.txt");
    std::ofstream f1(filepath.c_str());
    const auto& heightVal = _cloth.getHeightvals();

    for (auto ix : ixParticle) {
        auto* particle = _cloth.getParticle1d(ix);
        const auto& height = heightVal[ix];
        f1 << std::fixed  << particle->getPos()[0] << " "
           << particle->getPos()[1]<< " " << -height << std::endl;
    }

    f1.close();
}



