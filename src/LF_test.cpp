#include "ros/ros.h"
#include "std_msgs/String.h"

#include "line_following/LineFollowing.h"

#include <vector>
#include <cmath>

#include "Map_builder.h"
#include "Track_base.h"

using namespace std;

bool isRun = true;

MapBuilder theMap;

bool add(line_following::LineFollowing::Request  &req,
         line_following::LineFollowing::Response &res)
{
    int id = (int)req.id;
    double x = (double)req.x;
    double y = (double)req.y;
    ROS_INFO("Request: id=%d, x=%lf, y=%lf", id, x, y);

    if(id>=theMap.trackList.size())
    {
        res.res = 1;
        ROS_INFO("Failed to give result. Error code: 1 (track unregistered)");
    }
    else if(!theMap.trackList[id]->isWithinRange(x, y))
    {
        res.res = 2;
        ROS_INFO("Failed to give result. Error code: 2 (out of range)");
    }
    else
    {
        res.res = 0;
        theMap.trackList[id]->GetVector(x, y, res.dx, res.dy);
        double no = sqrt(res.dx*res.dx+res.dy*res.dy);
        res.dx = res.dx/no;
        res.dy = res.dy/no;

        ROS_INFO("Sending back response: [%lf, %lf]", res.dx, res.dy);
    }

    return true;
}

void cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Heard command: [%s]", msg->data.c_str());
    if(strcmp(msg->data.c_str(),"exit")==0) isRun = false;
}

int main(int argc, char **argv)
{
    theMap.BuildMap();

    ros::init(argc, argv, "lf_test");

    ros::NodeHandle n;

    ros::MultiThreadedSpinner spinner(20); // Use 4 threads

    ros::ServiceServer service = n.advertiseService("lf_grad", add);
    ROS_INFO("Ready to calculate gradients.");
    
    ros::Subscriber sub = n.subscribe("lf_cmd", 1000, cmdCallback);
    ROS_INFO("Welcome to line_following command center!");

    //ros::Rate r(10); // 10 hz

    //while(isRun)
    //{
    //    ros::spinOnce();
    //    r.sleep();
    //}

    spinner.spin();

    return 0;
}
