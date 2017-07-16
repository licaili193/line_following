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
}

void ArcTrack::SetCenter(double x, double y)
{
    _cX = x;
    _cY = y;
}

void ArcTrack::SetAngleRange(double s, double e)
{
    _angleS = s;
    _angleE = e;
}

void ArcTrack::GetVector(double x, double y, double &resX, double &resY)
{
    if(_isClockwise)
    {
        resX = y - 4*x*(x*x+y*y-1);
        resY = -x - 4*y*(x*x+y*y-1);
    }
    else
    {
        resX = -y - 4*x*(x*x+y*y-1);
        resY = x - 4*y*(x*x+y*y-1);
    }
}

bool ArcTrack::isWithinRange(double x, double y)
{
    double ang = atan2(y-_cY,x-_cX);
    if(ang<_angleS||ang>_angleE) return false;
    else
    {
        double r = sqrt((x-_cX)*(x-_cX)+(y-_cY)*(y-_cY));
        if(r<_radius-_width||r>_radius+_width) return false;
        else return true;
    }
}
