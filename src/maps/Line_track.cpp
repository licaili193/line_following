#include "Track_base.h"
#include "Line_track.h"

#include<cmath>

#include <iostream>

using namespace std;

LineTrack::LineTrack():TrackBase()
{
    _dirX = 1;
    _dirY = 0;
    _length = 1;
    _startX = _startY = 0;

    _para = 5;
}

void LineTrack::SetParameter(double p)
{
    _para = p;
}

void LineTrack::SetOrigin(double x, double y)
{
    _startX = x;
    _startY = y;
}

void LineTrack::SetDirection(double x, double y)
{
    _dirX = x;
    _dirY = y;

    if(x==0&&y==0) {x=1;y=0;}
    else
    {
        double norm = sqrt(x*x+y*y);
        _dirX = x/norm;
        _dirY = y/norm;
    }
}

void LineTrack::SetLength(double l)
{
    _length = l;
}

void LineTrack::GetVector(double x, double y, double &resX, double &resY)
{
    double dX = _startX-x-((_startX-x)*_dirX+(_startY-y)*_dirY)*_dirX;
    double dY = _startY-y-((_startX-x)*_dirX+(_startY-y)*_dirY)*_dirY;

    //double dl = sqrt(dX*dX+dY*dY);

    resX = _dirX + _para*dX;
    resY = _dirY + _para*dY;
}

bool LineTrack::isWithinRange(double x, double y)
{
    double dX = _startX-x-((_startX-x)*_dirX+(_startY-y)*_dirY)*_dirX;
    double dY = _startY-y-((_startX-x)*_dirX+(_startY-y)*_dirY)*_dirY;

    double dl = sqrt(dX*dX+dY*dY);
    //cout<<"Debug 1: "<<dl<<endl;

    if(dl>_width) return false;
    else
    {
        double endX = _startX + _length*_dirX;
        double endY = _startY + _length*_dirY;
        double midX = (_startX + endX)/2;
        double midY = (_startY + endY)/2;

        dX = midX-x+(-(midX-x)*_dirY+(midY-y)*_dirX)*_dirY;
        dY = midY-y-(-(midX-x)*_dirY+(midY-y)*_dirX)*_dirX;

        dl = sqrt(dX*dX+dY*dY);
        //cout<<"Debug 2: "<<dl<<endl;

        if(dl>_length/2) return false;
        else return true;
    }
}
