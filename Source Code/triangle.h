/********************************************
 * Class: Triangle
 * Description:xxx
 * Notice: xxx
 * Author: 于大宇
 * Site: WHU
 * Date: 20201207
********************************************/
#ifndef TRIANGLE_H
#define TRIANGLE_H

/**
 * @brief The Triangle class
 * 一个三角面片类
 */
class Triangle
{
public:
    ~Triangle();
    Triangle();
public:
    int _geomIndex;//该三角形在对应的顶点文件的索引
    int _vertexIndexs[3];//顶点索引
    float _normal[3];//法向量
    //std::vector<int> _neighborTriangles;//相邻的三角形的索引
};

#endif // TRIANGLE_H
