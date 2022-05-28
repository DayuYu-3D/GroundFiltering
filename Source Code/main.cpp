#include <string>
#include <iostream>
#include <chrono>
#include <map>

#include "cmdline.h"
#include "triangle.h"
#include "vertex.h"
#include "vec3d.h"
#include "trianglereader.h"
#include "yutils/Cfg.h"
#include "yutils/FileNameUtils.h"
#include "CSF/csf4tri.h"

int main(int argc, char *argv[])
{
    cmdline::parser a;
    a.add<std::string>("cfgPath", 'f', "The path of params.cfg. Default: ../params.cfg",
                       false, "../params.cfg");
    a.parse_check(argc, argv);
    std::string fp_cfg = a.get<std::string>("cfgPath");/// 获取输入的参数值

    //step0: 参数解析
    Cfg cfg;

    std::string slop_smooth;
    cfg.readConfigFile(fp_cfg.c_str(), "slop_smooth", slop_smooth);
    bool bPostPoress = cfg.readBoolByString(slop_smooth);

    std::string terrain_correct;
    cfg.readConfigFile(fp_cfg.c_str(), "terrain_correct", terrain_correct);
    bool bCorrectAbnormal = cfg.readBoolByString(terrain_correct);

    std::string class_threshold;
    cfg.readConfigFile(fp_cfg.c_str(), "class_threshold", class_threshold);
    std::string cloth_resolution;
    cfg.readConfigFile(fp_cfg.c_str(), "cloth_resolution", cloth_resolution);
    std::string iterations;
    cfg.readConfigFile(fp_cfg.c_str(), "iterations", iterations);
    std::string rigidness;
    cfg.readConfigFile(fp_cfg.c_str(), "rigidness", rigidness);
    std::string time_step;
    cfg.readConfigFile(fp_cfg.c_str(), "time_step", time_step);
    std::string angle2;
    cfg.readConfigFile(fp_cfg.c_str(), "angle2", angle2);
    std::string triangle_path;
    cfg.readConfigFile(fp_cfg.c_str(), "triangle_path", triangle_path);

    //step1: 文件解析
    std::vector<Triangle> vTriangle; ///大量内存占用
    std::map<size_t, std::vector<Vertex>> mvVertex; ///大量内存占用
    TriangleReader triReader(triangle_path);
    triReader.setPointFileFlag(true);
    if(!triReader.readTriangleData(vTriangle, mvVertex))
    {
        std::cout << "read data failed, quit!" << std::endl;
        return -1;
    }

    //获取开始时间
    auto startT = std::chrono::system_clock::now();

    //step2: 初始化CSF4Tri
    Params ps;
    ps.bSloopSmooth = bPostPoress;
    ps.classThreshold = atof(class_threshold.c_str());
    ps.clothResolution = atof(cloth_resolution.c_str());
    ps.interations = atoi(iterations.c_str());
    ps.rigidness = atoi(rigidness.c_str());
    ps.timeStep = atof(time_step.c_str());
    ps.angle2 = atoi(angle2.c_str());
    ps.bCorrectAbnormal = bCorrectAbnormal;

    CSF4Tri csf4tri;
    csf4tri.setTriangleCloud(&vTriangle, &mvVertex);///顶点和面片设置
    csf4tri.setParams(ps);///参数设置

    //step3: 过滤
    std::vector<unsigned int> gd, offgd;
    csf4tri.doCSFFilter(gd, offgd);

    //获取结束时间
    auto entT = std::chrono::system_clock::now();
    double ut = std::chrono::duration_cast<std::chrono::milliseconds>(entT - startT).count();
    std::cout << "* Total use time: " << (double)ut / 1000 << "s" << std::endl;

    //step4: 保存结果
    std::string strFP = yUtils::getNameLessExtension(triangle_path);
    std::string strFP_ground = strFP + "_ground.txt";
    std::string strFP_offGround = strFP + "_offGround.txt";
    std::cout << "Saving file..." << std::endl;
    std::cout << "Save ground path: " << strFP_ground << std::endl;
    triReader.writeTrianglesData(vTriangle, gd, strFP_ground);
    std::cout << "Save offGround poth: " << strFP_offGround << std::endl;
    triReader.writeTrianglesData(vTriangle, offgd, strFP_offGround);
    std::cout << "Save done!" << std::endl;

    return 0;
}
