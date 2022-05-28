/********************************************
 * Class: Cloth
 * Description:xxx
 * Notice: xxx
 * Author: 于大宇
 * Site: WHU
 * Date: 20201207
********************************************/
#ifndef CLOTH_H
#define CLOTH_H

#include <vector>
#include <string>

#include "vec3d.h"
#include "particle.h"

// post processing is only for connected component which is large than 50
#define MAX_PARTICLE_FOR_POSTPROCESSIN    100

struct ij {
    ij(int x1, int y1) {
        i = x1;
        j = y1;
    }

    int i; //为length,行数
    int j; //为width,列数
};

class Cloth
{
public:
    Cloth(const Vec3d &originPos,
          int numWidth_particles,
          int numLength_particles,
          double stepX,
          double stepY,
          double smoothThreshold,
          double altitudeThreshold,
          int rigidness,
          double timeStep,
          bool bcorrectAbnormal,
          int angle2);

    /* This includes calling satisfyConstraint() for every constraint, and calling
     * timeStep() for all particles
     */
    double timeStep();

    /* used to add gravity (or any other arbitrary vector) to all particles */
    void addForce(const Vec3d direction);

    /**
     * @brief terrCollision: 碰撞检测
     */
    void terrCollision();

    void movableFilter();

    std::vector<int> findUnmovablePoint(std::vector<ij> connected);

    void handleSlopConnected(std::vector<int> edgePoints,
                             std::vector<ij> connected,
                             std::vector<std::vector<int> > neibors);

    void saveToFile(std::string path = "");

    void saveMovableToFile(std::string path = "");

    int getSize() {
        return _numLength_particles * _numWdith_particles;
    }
    std::size_t get1DIndex(int x, int y) {
        return y * _numWdith_particles + x;
    }
    inline std::vector<double> &getHeightvals() {
        return _vHeightvals;
    }
    Particle *getParticle1d(int index) {
        return &_vParticles[index];
    }
    Particle *getParticle(int i, int j);

protected:
    void makeConstraint(Particle *p1, Particle *p2);
private:
    // particles总数
    int _constraintIterations;
    int _rigidness;
    double _timeStep;
    std::vector<Particle> _vParticles; //all particles
    double _smoothThreshold;
    double _altitudeThreshold;

public:
    Vec3d _originPos;
    double _stepX;
    double _stepY;
    std::vector<double> _vHeightvals; //对应的height valules
    int _numWdith_particles; //列数，x/间隔
    int _numLength_particles;//行数，y/间隔

    bool _bcorrectAbnomal;
    int _angle2; //用于异常粒子检测的角度阈值，取tan(_angle2)
};

#endif // CLOTH_H
