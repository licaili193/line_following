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
    //source 0
    LineTrack* lt1 = new LineTrack;//S94
    lt1->SetWidth(0.2286);
    lt1->SetOrigin(2.4352,1.2192);
    lt1->SetDirection(-1,0);
    lt1->SetLength(3.3096+0.5);
    lt1->SetParameter(2);

    trackList.push_back(lt1);

    //source 1
    ArcTrack* lt2 = new ArcTrack;//A106
    lt2->SetCenter(-0.762,0.-2.6924);
    lt2->SetRadius(3.9115);
    lt2->SetAngleRange(-1.5708,-1.20428);
    lt2->SetWidth(0.2286);
    lt2->SetDirection(false); //false is ccw
    lt2->SetRangeDirection(false);
    lt2->SetParameter(.2);

    trackList.push_back(lt2);

    //source 2
    ArcTrack* lt3 = new ArcTrack;//A105-A101-A100
    lt3->SetCenter(-2.032,0.3556);
    lt3->SetRadius(0.6096);
    lt3->SetAngleRange(-1.172861,3.0102);
    lt3->SetWidth(0.2286);
    lt3->SetDirection(false);
    lt3->SetRangeDirection(false);
    lt3->SetParameter(.2);

    trackList.push_back(lt3);

    //source 3
    ArcTrack* lt4 = new ArcTrack;//A99
    lt4->SetCenter(-0.8744,0.2042);
    lt4->SetRadius(0.5574);
    lt4->SetAngleRange(-3.1414,-1.5708);// I am not sure about the order of this !(Behdad)
    lt4->SetWidth(0.2286);
    lt4->SetDirection(true);
    lt4->SetRangeDirection(false);
    lt4->SetParameter(.2);

    trackList.push_back(lt4);

    //source 4
    LineTrack* lt5 = new LineTrack;//S95-S91-S89 (just had to increase length to combine)
    lt5->SetWidth(0.2286);
    lt5->SetOrigin(-0.8744,0.7616);
    lt5->SetDirection(1,0);
    lt5->SetLength(2.9945);
    lt5->SetParameter(2);
    
    trackList.push_back(lt5);

    //source 5
    ArcTrack* lt6 = new ArcTrack;//A92
    lt6->SetCenter(1.0676,0.1286);
    lt6->SetRadius(0.6877);
    lt6->SetAngleRange(-1.6650,-2.7541);
    lt6->SetWidth(0.2286);
    lt6->SetDirection(true);
    lt6->SetRangeDirection(false);
    lt6->SetParameter(0.2);

    trackList.push_back(lt6);    

    //source 6
    ArcTrack* lt7 = new ArcTrack;//A87-84-81
    lt7->SetCenter(2.2352,0.6096);
    lt7->SetRadius(0.6096);
    lt7->SetAngleRange(0.3194,1.1694); //origially -4.5,-1.5708
    lt7->SetWidth(0.2286);
    lt7->SetDirection(false);
    lt7->SetRangeDirection(false);
    lt7->SetParameter(0.2);

    trackList.push_back(lt7); 

 //Road 2
    //source 7
    LineTrack* lt8 = new LineTrack;//S61
    lt8->SetWidth(0.2286);
    lt8->SetOrigin(2.0066,-0.970);
    lt8->SetDirection(-1,0);
    lt8->SetLength(1.6256);
    lt8->SetParameter(2);
    
    trackList.push_back(lt8);

    //source 8
    ArcTrack* lt9 = new ArcTrack;//A58
    lt9->SetCenter(0.34,-0.8);
    lt9->SetRadius(0.1224);
    lt9->SetAngleRange(1.5708,-3.0);
    lt9->SetWidth(0.2286);
    lt9->SetDirection(true);
    lt9->SetRangeDirection(false);
    lt9->SetParameter(0.9);

    trackList.push_back(lt9); 

    //source 9
    LineTrack* lt10 = new LineTrack;//S80
    lt10->SetWidth(0.2286);
    lt10->SetOrigin(0.2150,-0.8082);
    lt10->SetDirection(0,1);
    lt10->SetLength(1.343);
    lt10->SetParameter(2);
    
    trackList.push_back(lt10);

    //source 10
    ArcTrack* lt11 = new ArcTrack;//A94
    lt11->SetCenter(0.6858,0.3048);
    lt11->SetRadius(0.4572);
    lt11->SetAngleRange(3.1414,-2.7244);
    lt11->SetWidth(0.2286);
    lt11->SetDirection(true);
    lt11->SetRangeDirection(false);
    lt11->SetParameter(0.2);

    trackList.push_back(lt11); 

    //source
    //S95-S91-S89 (just had to increase length to combine)

    //source
    //A92  

    //source
    //A87-84-81

    //source 11
    ArcTrack* lt12 = new ArcTrack;//A86
    lt12->SetCenter(1.82,-0.2032);
    lt12->SetRadius(0.2763); //0.2763
    lt12->SetAngleRange(-1.16,0.0);
    lt12->SetWidth(0.2286);
    lt12->SetDirection(false);
    lt12->SetRangeDirection(false);
    lt12->SetParameter(0.2);

    trackList.push_back(lt12); 

    //source 12
    LineTrack* lt13 = new LineTrack;//S88
    lt13->SetWidth(0.2286);
    lt13->SetOrigin(2.159,.1032);
    lt13->SetDirection(0,-1);
    lt13->SetLength(0.935);
    lt13->SetParameter(2);
    
    trackList.push_back(lt13);

    //source 13
    ArcTrack* lt14 = new ArcTrack;//A53
    lt14->SetCenter(2.0066,-0.8382);
    lt14->SetRadius(0.1524);
    lt14->SetAngleRange(0.0,1.2508);
    lt14->SetWidth(0.2286);
    lt14->SetDirection(true);
    lt14->SetRangeDirection(false);
    lt14->SetParameter(0.2);

    trackList.push_back(lt14); 

    //source 14
    ArcTrack* lt15 = new ArcTrack;//A87-84-81
    lt15->SetCenter(2.2352,0.6096);
    lt15->SetRadius(0.6096);
    lt15->SetAngleRange(1.1694,2.5656); //origially -4.5,-1.5708
    lt15->SetWidth(0.2286);
    lt15->SetDirection(false);
    lt15->SetRangeDirection(false);
    lt15->SetParameter(0.2);

    trackList.push_back(lt15);

    //source 15
    ArcTrack* lt16 = new ArcTrack;//A87-84-81
    lt16->SetCenter(2.2352,0.6096);
    lt16->SetRadius(0.6096);
    lt16->SetAngleRange(2.5656,-1.5708); //origially -4.5,-1.5708
    lt16->SetWidth(0.2286);
    lt16->SetDirection(false);
    lt16->SetRangeDirection(false);
    lt16->SetParameter(0.2);

    trackList.push_back(lt16); 
}