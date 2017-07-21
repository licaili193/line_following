#include "Track_base.h"
#include "Arc_track.h"

#include<cmath>

using namespace std;

ArcTrack::ArcTrack():TrackBase()
{
    _cX = 0;
    _cY = 0;
    _radius = 1;
    _angleS = _angleE = 0;

    _isClockwise = true;
    _isInner = true;
}

void ArcTrack::SetCenter(double x, double y)
{
    _cX = x;
    _cY = y;
}

void ArcTrack::SetAngleRange(double s, double e)
{
    _angleS = s<0?s+2*PI:s;
    _angleE = e<0?e+2*PI:e;
}

void ArcTrack::SetRadius(double l)
{
    _radius = l;
}

void ArcTrack::GetVector(double x, double y, double &resX, double &resY)
{
    if(_isClockwise)
    {
        resX = _radius*(y-_cY) - 4*(x-_cX)*((x-_cX)*(x-_cX)+(y-_cY)*(y-_cY)-_radius*_radius);
        resY = -_radius*(x-_cX) - 4*(y-_cY)*((x-_cX)*(x-_cX)+(y-_cY)*(y-_cY)-_radius*_radius);
    }
    else
    {
        resX = -_radius*(y-_cY) - 4*(x-_cX)*((x-_cX)*(x-_cX)+(y-_cY)*(y-_cY)-_radius*_radius);
        resY = _radius*(x-_cX) - 4*(y-_cY)*((x-_cX)*(x-_cX)+(y-_cY)*(y-_cY)-_radius*_radius);
    }
}

void ArcTrack::SetDirection(bool c)
{
    _isClockwise = c;
}

void ArcTrack::SetRangeDirection(bool c)
{
    _isInner = c;
}

bool ArcTrack::isWithinRange(double x, double y)
{
    double ang = atan2(y-_cY,x-_cX);
    ang = ang<0?ang+2*PI:ang;
    if(_isInner) {if(ang<_angleS||ang>_angleE) return false;}
    else {if(ang>_angleS&&ang<_angleE) return false;}
    double r = sqrt((x-_cX)*(x-_cX)+(y-_cY)*(y-_cY));
    if(r<_radius-_width||r>_radius+_width) return false;
    else return true;
}
