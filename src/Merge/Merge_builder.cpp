#include "Merge_builder.h"
#include "Control_zone.h"

#include <queue>
#include <ctime>
#include <tuple>

using namespace std;

MergeBuilder::MergeBuilder()
{
    zoneList.clear();
}

MergeBuilder::~MergeBuilder()
{
    while(!zoneList.empty())
    {
        delete zoneList[zoneList.size()-1];
        zoneList.pop_back();
    }
}

void MergeBuilder::BuildMerge()
{
    ControlZone* z = new ControlZone(0);
    z->init_time = time(0);
    z->entrences.push_back(EntrenceInfo{1.376,0.3048,0.3});
    z->entrences.push_back(EntrenceInfo{1.376,0.3048,0.3});
    zoneList.push_back(z);
}