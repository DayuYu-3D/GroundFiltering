/********************************************
 * Class: ParticalAltitudeFinder
 * Description: xxx
 * Notice: xxx
 * Author: 于大宇
 * Site: WHU
 * Date: 20210817
********************************************/
#ifndef PARTICALALTITUDEFINDER_H
#define PARTICALALTITUDEFINDER_H

#include <map>

#include "cloth.h"
#include "triangle.h"
#include "vertex.h"

#include <FastBVH.h> //path: Fast-BVH\include

#include "BVHIntersector.h"

class ParticalAltitudeFinder
{
public:
    ParticalAltitudeFinder(Cloth &cloth,
                           const std::vector<Triangle> *pvTriangle,
                           const std::map<size_t, std::vector<Vertex>> *pvvVertex);
    ~ParticalAltitudeFinder();

    void findIntersectionHeight();

private:

    /**
     * @brief findAbnormalParticles_nearstVertex: 为粒子寻找碰撞的高程值, 通过最邻近顶点
     */
    void findCollsionHeight_nearstVertex();

    /**
     * @brief findCollsionHeight_bvh: 为粒子寻找碰撞的高程值, 通过BVH的ray和face求交
     */
    void findCollsionHeight_bvh();

    // for a cloth particle, if no corresponding triangle are found.
    // the heightval are set as its neighbor's
    double findHeightValByNeighbor(Particle *p);
    /**
     * @brief findHeightValByScanline：从他的上下左右出发，找到一个有效高程值的邻居，就return
     * @param p
     * @param cloth
     * @return：返回的邻居的有效高程值
     */
    double findHeightValByScanline(Particle *p, Cloth &cloth);

    /**
     * @brief findAbnormalParticles: 寻找异常粒子
     */
    void findAbnormalParticles();

    /**
     * @brief saveParticlesAndHeight: 保存寻找到的交点高程值看看
     */
    void saveParticlesAndHeight();

    /**
     * @brief saveAbnormalParticleAndHeight: 保存寻找到的异常的交点高程值看看
     */
    void saveAbnormalParticleAndHeight(std::vector<size_t>& ixParticle);

private:
    Cloth &_cloth;
    const std::vector<Triangle> *_pvTriangle;
    const std::map<size_t, std::vector<Vertex>> *_pmvVertex;

    std::vector<std::uint32_t> _face_indices;
    FastBVH::BVH<float, std::uint32_t>* _pBVH; //大内存占用
    FastBVH::Traverser<float, uint32_t, FaceIntersector<float>>* _pTraverser;

};


#endif // PARTICALALTITUDEFINDER_H
