/********************************************
 * Class: Vertex
 * Author: Dayu
 * Site: WHU
 * Date: 20201207
********************************************/
#ifndef VERTEX_H
#define VERTEX_H

#include "vec3d.h"
/**
 * 顶点类，记录顶点坐标、法向量、纹理坐标等
 */
class Vertex
{
public:
    Vertex();
    ~Vertex();

public:
    int _index;//该顶点在数组中的下标
    Vec3d _coor;//顶点坐标
    float _texCoor[2];//纹理坐标
    //std::vector<int> _neighborTriangle;//顶点相邻的三角形
};

#endif // VERTEX_H
