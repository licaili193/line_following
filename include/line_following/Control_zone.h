#ifndef __CONTROLZONE
#define __CONTROLZONE

#include <queue>
#include <vector>

using namespace std;

struct ControlInfo
{
    int robot_id;
    int entrence_id;
    double speed;
    double t0;
    double tm;
    double tf;
};

struct EntrenceInfo
{
    double L;
    double S;
    double delta;
};

class ControlZone
{
    int id;
public:
    ControlZone(int d) {id = d;}
    queue<ControlInfo>q;
    double init_time;
    vector<EntrenceInfo>entrences;
};

#endif