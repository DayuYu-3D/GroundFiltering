#ifndef SPLITTRIANGLES2TERRAIN_H
#define SPLITTRIANGLES2TERRAIN_H

#include <map>

#include "cloth.h"
#include "triangle.h"
#include "vertex.h"

class SplitTriangles2Terrain
{
public:
    SplitTriangles2Terrain(double threshold);
    ~SplitTriangles2Terrain();

    void splitTerrain(Cloth &cloth,
                      const std::vector <Triangle> *pvTriangle,
                      const std::map<size_t, std::vector<Vertex>> *pmvVertex,
                      std::vector<unsigned int> &groundIndexes,
                      std::vector<unsigned int> &offGroundIndexes);

    /**
     * @brief splitTerrain_byBVH: 通过bvh树分离，看看有没有提升
     * @param cloth
     * @param pvTriangle
     * @param pvvVertex
     * @param groundIndexes: 地面三角形索引
     * @param offGroundIndexes: 非地面三角形索引
     */
    void splitTerrain_byBVH(Cloth &cloth,
                      const std::vector <Triangle> *pvTriangle,
                      const std::map<size_t, std::vector<Vertex>> *pmvVertex,
                      std::vector<unsigned int> &groundIndexes,
                      std::vector<unsigned int> &offGroundIndexes);
private:
    double _classThreshold;
};

#endif // SPLITTRIANGLES2TERRAIN_H
