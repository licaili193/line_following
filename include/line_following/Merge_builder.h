#ifndef __MERGEBUILDER
#define __MERGEBUILDER

#include "Control_zone.h"

#include <vector>

using namespace std;

class MergeBuilder
{
public:
    MergeBuilder();
    ~MergeBuilder();

    vector<ControlZone*>zoneList;

    void BuildMerge();
};

#endif