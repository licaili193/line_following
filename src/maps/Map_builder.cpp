#include "Map_builder.h"

#include "Track_base.h"
#include "Line_track.h"
#include "Arc_track.h"

#include <vector>

using namespace std;

MapBuilder::MapBuilder()
{
    trackList.clear();
}

MapBuilder::~MapBuilder()
{
    while(!trackList.empty())
    {
        delete trackList[trackList.size()-1];
        trackList.pop_back();
    }
}

void MapBuilder::BuildMap()
{
    LineTrack* lt1 = new LineTrack;
    lt1->SetWidth(0.124);
    lt1->SetOrigin(-0.08,-0.08);
    lt1->SetDirection(-1,0);
    lt1->SetLength(0.7);
    lt1->SetParameter(10);

    trackList.push_back(lt1);
}