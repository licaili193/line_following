#ifndef __TRACKBASE
#define __TRACKBASE

virtual class TrackBase{
    double _width; 
public:
    TrackBase();

    void SetWidth(double w);
    double GetWidth();
    virtual bool isWithinRange(double x, double y);
    virtual void GetVector(double x, double y, double &resX, double &resY);
};

#endif
