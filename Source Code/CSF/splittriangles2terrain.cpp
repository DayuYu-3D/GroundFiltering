#include "splittriangles2terrain.h"

#include <iostream>

SplitTriangles2Terrain::SplitTriangles2Terrain(double threshold)
    : _classThreshold(threshold)
{

}

SplitTriangles2Terrain::~SplitTriangles2Terrain()
{

}

void SplitTriangles2Terrain::splitTerrain(Cloth &cloth,
        const std::vector<Triangle> *pvTriangle,
        const std::map<size_t, std::vector<Vertex> > *pmvVertex,
        std::vector<unsigned int> &groundIndexes,
        std::vector<unsigned int> &offGroundIndexes)
{
    groundIndexes.resize(0);
    offGroundIndexes.resize(0);

    //第一种方法:根据三角形顶点分离,
    int nTris = static_cast<int>(pvTriangle->size());
    for(int iTri = 0; iTri < nTris; ++iTri)
    {
        auto gemIx = pvTriangle->at(iTri)._geomIndex;
        for(int j = 0; j < 3; ++j)
        {
            auto pointIx = pvTriangle->at(iTri)._vertexIndexs[j];
            Vertex point = pmvVertex->at(gemIx)[pointIx];
            double x = point._coor[0];
            double y = point._coor[1];

            double deltaX = x - cloth._originPos[0];
            double deltaY = y - cloth._originPos[1];

            int col0 = int(deltaX / cloth._stepX);
            int row0 = int(deltaY / cloth._stepY);
            int col1 = col0 + 1;
            int row1 = row0;
            int col2 = col0 + 1;
            int row2 = row0 + 1;
            int col3 = col0;
            int row3 = row0 + 1;

            double subdeltaX = (deltaX - col0 * cloth._stepX) / cloth._stepX;
            double subdeltay = (deltaY - row0 * cloth._stepY) / cloth._stepY;

            //四点加权距离
//            std::cout << cloth.getParticle(row0, col0)->_pos[2] << "   ";
//            std::cout << cloth.getParticle(row3, col3)->_pos[2] << "   ";
            double fxy = cloth.getParticle(row0, col0)->_pos[2] * (1 - subdeltaX) * (1 - subdeltay) +
                         cloth.getParticle(row3, col3)->_pos[2] * (1 - subdeltaX) * subdeltay +
                         cloth.getParticle(row2, col2)->_pos[2] * subdeltaX * subdeltay +
                         cloth.getParticle(row1, col1)->_pos[2] * subdeltaX * (1 - subdeltay);
            double height_var = fxy - point._coor[2];

            //如果三个顶有一个点是非地面点,则认为该三角形为非地面三角形
            if (fabs(height_var) > _classThreshold) {
                offGroundIndexes.push_back((unsigned int)iTri);
                break;
            } else {
                if(j == 2)
                {
                    //如果三个顶点都是地面点,则认为该三角形为地面三角形
                    groundIndexes.push_back((unsigned int)iTri);
                }
            }
//            //如果有一个顶点是地面点,则认为该三角形为地面三角形
//            if (fabs(height_var) < _classThreshold) {
//                groundIndexes.push_back((unsigned int)iTri);
//                break;
//            } else {
//                if(j == 2)
//                {
//                    //如果三个顶点都不是地面点,则认为该三角形为非地面三角形
//                    offGroundIndexes.push_back((unsigned int)iTri);
//                }
//            }
        }
    }
}

void SplitTriangles2Terrain::splitTerrain_byBVH(Cloth &cloth, const std::vector<Triangle> *pvTriangle,
        const std::map<size_t, std::vector<Vertex> > *pmvVertex,
        std::vector<unsigned int> &groundIndexes,
        std::vector<unsigned int> &offGroundIndexes)
{

}

