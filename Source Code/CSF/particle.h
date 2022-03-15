#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

#include "vec3d.h"

/*
Initially, we have to make modifications of particle positions for each constraint(constraintTimes = rigidness), However, to save computation time, we
precomputed the total displacement of a particle for all constraintTimes.
For singleMove1, which means one of the two particles is unmovable, then we move the other one only:
if constraintTimes = 0: singleMove1 = 0
if constraintTimes = 1: singleMove1 = 0.3, i.e., each time we move 0.3 (scale factor of the total distance) for a particle towards the other one
if constraintTimes = 2: singleMove1 = (1-0.3)*0.3+0.3 = 0.51
if constraintTimes = 3: singleMove1 = (1-0.51)*0.3+0.51 = 0.657
...

For doubleMove1, we move both of the two particles towards each other.
if constraintTimes = 0: singleMove2 = 0
if constraintTimes = 1: singleMove2 = 0.3, i.e., each time we move 0.3 (scale factor of the total distance) for the two particles towards each other
if constraintTimes = 2: singleMove2 = (1-0.3*2)*0.3+0.3 = 0.42
if constraintTimes = 3: singleMove2 = (1-0.42*2)*0.3+0.42 = 0.468
...
*/
const double singleMove1[15] = { 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
const double doubleMove1[15] = { 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };
#define DAMPING    0.01 // 阻尼量,how much to damp the cloth simulation each frame
#define MAX_INF    9999999999
#define MIN_INF    -9999999999

class Particle
{
public:
    Particle();
    Particle(Vec3d pos, double timeStep);

    inline bool isMovable() {
        return _movable;
    };

    inline void addForce(Vec3d f)
    {
        _acceleration = f / _mass;
    };

    /**
     * @brief 主要函数: 给定方程 "force = mass * acceleration"
     *  通过顶点积分计算下一点位置*
     */
    inline void timeStep() {
        if (_movable) {
            Vec3d temp = _pos;
            double ddd = 1.0 - DAMPING;
            _pos = _pos + (_pos - _oldPos) * ddd + _acceleration * _timeStep2;
            _oldPos = temp;
        }
    };

    inline Vec3d &getPos() {
        return _pos;
    };

    inline void resetAcceleartion() {
        _acceleration = Vec3d(0.0, 0.0, 0.0);
    };

    inline void offsetPos(const Vec3d v) {
        if (_movable) {
            _pos += v;
        }
    };

    inline void setunMovable() {
        _movable = false;
    };

    inline void addToNormal(Vec3d normal) {
        _accumulatedNormal += normal.normalize();
    };

    inline Vec3d &getNormal() {
        // note: The normal is not unit length
        return _accumulatedNormal;
    };

    inline void resetNormal() {
        _accumulatedNormal = Vec3d(0, 0, 0);
    };

    /**
     * @brief satisfyConstraintSelf 内部力
     * @param constraintTimes
     */
    void satisfyConstraintSelf(int constraintTimes);

private:
    bool _movable;            // 该粒子是否可移动
    double _mass;             // 该粒子的质量,被设为常数1
    Vec3d _acceleration;       // 粒子当前的加速度,向量
    Vec3d _accumulatedNormal; // 加速度的法线 (i.e. non normalized), used for OpenGL soft shading
    double _timeStep2; //时间步长的平方
public:
    // 这两个变量在布料模拟步骤后的边缘平滑过程中使用。
    Vec3d _pos;     // 粒子目前位置
    //粒子在前一时间步中的位置，作为小数点数值积分方案的一部分。
    Vec3d _oldPos;
    bool _isVisited;
    int _posJ; // 列数 width
    int _posI; //行数 length
    int _cPos;

    std::vector<Particle *> _neighborsList;

    std::vector<int> _correspondingLidarPointList;//TODO:需更改
    std::size_t _nearestPointIndex; //TODO:需更改
    double _nearestPointHeight;
    double _tmpDist;
};

#endif // PARTICLE_H
