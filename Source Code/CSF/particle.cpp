#include "particle.h"

Particle::Particle():
    _movable(true),
    _mass(1),
    _acceleration(Vec3d(0, 0, 0)),
    _accumulatedNormal(Vec3d(0, 0, 0)),
    _timeStep2(0.0),
    _isVisited(false),
    _posJ(0),
    _posI(0),
    _cPos(0),
    _nearestPointHeight(MIN_INF),
    _tmpDist(MAX_INF)
{

}

Particle::Particle(Vec3d pos, double timeStep) :
    Particle()
{
    _pos = pos;
    _oldPos = pos;
    _timeStep2 = timeStep;
}


void Particle::satisfyConstraintSelf(int constraintTimes)
{
    Particle *p1 = this;

    for (std::size_t i = 0; i < _neighborsList.size(); i++) {
        Particle *p2 = _neighborsList[i];
        Vec3d correctionVector(0, 0, p2->_pos[2] - p1->_pos[2]);

        if (p1->isMovable() && p2->isMovable()) {
            // Lets make it half that length, so that we can move BOTH p1 and p2.
            double ddd = constraintTimes > 14 ? 0.5 : doubleMove1[constraintTimes];
            Vec3d correctionVectorHalf = correctionVector * ddd;
            p1->offsetPos(correctionVectorHalf);
            p2->offsetPos(-correctionVectorHalf);
        } else if (p1->isMovable() && !p2->isMovable()) {
            double ddd = constraintTimes > 14 ? 1 : singleMove1[constraintTimes];
            Vec3d correctionVectorHalf = correctionVector * ddd;
            p1->offsetPos(correctionVectorHalf);
        } else if (!p1->isMovable() && p2->isMovable()) {
            double ddd = constraintTimes > 14 ? 1 : singleMove1[constraintTimes];
            Vec3d correctionVectorHalf = correctionVector * ddd;
            p2->offsetPos(-correctionVectorHalf);
        }
    }
}
