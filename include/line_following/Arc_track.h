#ifndef __ARCTRACK
#define __ARCTRACK

#include "Track_base.h"

#define PI 3.141592653

class ArcTrack: public TrackBase
{
    double _cX;
    double _cY;

    bool _isClockwise;

    double _radius;
    double _angleS;
    double _angleE;

public:
    ArcTrack();

    void SetCenter(double x, double y);
    void SetRadius(double l);
    void SetAngleRange(double s, double e);
};

#endif
