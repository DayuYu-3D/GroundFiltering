#ifndef TRIANGLEREADER_H
#define TRIANGLEREADER_H

#include <string>
#include <vector>
#include <map>

#include "triangle.h"
#include "vertex.h"

class TriangleReader
{
public:
    TriangleReader(std::string triFP);

    /**
     * @brief 快速解析顶点和三角面片文件夹
     * @param vTri: 需要分配的三角面片矢量
     * @param vvVertex: 需要分配的顶点二维矢量
     * @return 解析成功,返回true
     */
    bool readTriangleData(std::vector<Triangle> &vTri,
                          std::map<size_t, std::vector<Vertex>>  &mvVertex);

    /**
     * @brief readTrianglesData: 写triangle文件
     * @param vTri: vector of triangle,
     */
    void writeTrianglesData(const std::vector<Triangle> &vTri,
                            const std::vector<unsigned int> &vTriIndex,
                            std::string triP) const;

    void setPointFileFlag(bool b1PointFiel);
private:
    /**
     * @brief 检查文件完整性
     * @return 完整返回true,否则else
     */
    bool fileValidate();

    /**
     * @brief 预先为顶点和triangle矢量分配内存,防止内存指数型增长
     * @param vTri: 需要分配的三角面片矢量
     * @param vvVertex: 需要分配的顶点二维矢量
     */
    void allocateMemory(std::vector<Triangle> &vTri);
private:

    std::string _triFP;

    bool _b1PointFile; //point文件是否只有一个
};

#endif // TRIANGLEREADER_H
