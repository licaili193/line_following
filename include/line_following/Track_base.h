#ifndef __TRACKBASE
#define __TRACKBASE

virtual class TrackBase{
    double _width; 
public:
    void SetWidth(double w);
    double GetWidth();
    virtual bool isCatched(double x, double y);
    virtual bool isWithinRange(double x, double y);
    virtual void GetVector(double x, double y, double &resX, double &resY);
};

#endif
