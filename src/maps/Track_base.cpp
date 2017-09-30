#include "Track_base.h"

TrackBase::TrackBase()
{
    _width = 0;
}

void TrackBase::SetWidth(double w)
{
    _width = w;
}

double TrackBase::GetWidth()
{
    return _width;
}
