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
    lt1->SetWidth(0.2286);
    lt1->SetOrigin(0.4312,-0.3233);
    lt1->SetDirection(-1,1);
    lt1->SetLength(1.1430);
    lt1->SetParameter(5);

    trackList.push_back(lt1);

    ArcTrack* lt2 = new ArcTrack;
    lt2->SetCenter(0,0.7543);
    lt2->SetRadius(0.4372);
    lt2->SetAngleRange(-2.3562,-0.7854);
    lt2->SetWidth(0.2286);
    lt2->SetDirection(true);
    lt2->SetRangeDirection(false);

    trackList.push_back(lt2);

    LineTrack* lt3 = new LineTrack;
    lt3->SetWidth(0.2286);
    lt3->SetOrigin(0.3233,0.4312);
    lt3->SetDirection(-1,-1);
    lt3->SetLength(1.1430);
    lt3->SetParameter(5);
    
    trackList.push_back(lt3);

    ArcTrack* lt4 = new ArcTrack;
    lt4->SetCenter(0.02,-0.7543);
    lt4->SetRadius(0.6096);
    lt4->SetAngleRange(0.8029,2.3562);
    lt4->SetWidth(0.2286);
    lt4->SetDirection(false);
    lt4->SetRangeDirection(false);

    trackList.push_back(lt4);
}