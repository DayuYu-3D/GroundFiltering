/********************************************
 * Class: CSF4Tri
 * Description: 用于三角面片的CSF过滤器
 * Notice: xxx
 * Author: 于大宇
 * Site: WHU
 * Date: 20201207
********************************************/
#ifndef CSF4TRI_H
#define CSF4TRI_H

#include <vector>
#include <string>
#include <map>

#include "triangle.h"
#include "vertex.h"

/**
 * @brief The Params struct for CSF params
 */
struct Params
{
    bool bSloopSmooth = true;
    double timeStep = 0.65;
    double classThreshold = 0.5;
    double clothResolution = 1;
    int rigidness = 3;
    int interations = 500;
    int angle2 = 45;
    bool bCorrectAbnormal = true;
};

class CSF4Tri
{
public:
    CSF4Tri();
    ~CSF4Tri();

    /**
     * @brief setTriangleCloud
     * @param vTri: 三角面片,一维,指针,
     * @param vvVertex: 对应的顶点,二维,指针
     */
    void setTriangleCloud(const std::vector<Triangle> *vTri,
                          const std::map<size_t, std::vector<Vertex>> *vvVertex);
    /**
     * @brief setParams
     * @param ps: 默认见struct
     */
    void setParams(const Params ps);

    void doCSFFilter(std::vector<unsigned int> &groundIndexs,
                     std::vector<unsigned int> &offGroundIndexs) const;
protected:
    /**
     * @brief 计算所有顶点的xyz的最大值和最小值
     * @param bbmin
     * @param bbmax
     */
    void computeBoundingBox(Vec3d &bbmin, Vec3d &bbmax) const;
private:
    Params _params;
    const std::vector<Triangle> *_pVTri;
    const std::map<size_t, std::vector<Vertex>> *_mvVertex;
};

#endif // CSF4TRI_H
