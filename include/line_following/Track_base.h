#ifndef __TRACKBASE
#define __TRACKBASE

class TrackBase{
protected:
    double _width; 
public:
    TrackBase();

    void SetWidth(double w);
    double GetWidth();
    virtual bool isWithinRange(double x, double y) = 0;
    virtual void GetVector(double x, double y, double &resX, double &resY) = 0;
};

#endif
