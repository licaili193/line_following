#ifndef __ARCTRACK
#define __ARCTRACK

#include "Track_base.h"

#define PI 3.141592653

class ArcTrack: public TrackBase
{
    double _cX;
    double _cY;

    bool _isClockwise;
    bool _isInner;

    double _radius;
    double _angleS;
    double _angleE;

    double _para;

public:
    ArcTrack();

    void SetCenter(double x, double y);
    void SetRadius(double l);
    void SetParameter(double p);
    void SetAngleRange(double s, double e);
    void GetVector(double x, double y, double &resX, double &resY);
    bool isWithinRange(double x, double y);
    void SetDirection(bool c);
    void SetRangeDirection(bool c);
};

#endif
