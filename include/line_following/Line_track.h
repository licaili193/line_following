#ifndef __LINETRACK
#define __LINETRACK

#include "Track_base.h"

class LineTrack: public TrackBase
{
    double _startX;
    double _startY;

    double _dirX;
    double _dirY;

    double _length;

    double _para;
public:
    LineTrack();

    void SetParameter(double p);

    void SetOrigin(double x, double y);
    void SetDirection(double x, double y);
    void SetLength(double l);
};

#endif
