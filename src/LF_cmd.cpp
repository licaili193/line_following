#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lf_cmd");

    ros::NodeHandle n;

    ros::Publisher cmd_pub = n.advertise<std_msgs::String>("lf_cmd", 1000);

    while (ros::ok())
    {
   
        std_msgs::String msg;

        cout<<"Please input the command:"<<endl;

        string ss;
        cin>>ss;
        msg.data = ss.c_str();

        ROS_INFO("%s", msg.data.c_str());
        cmd_pub.publish(msg);

        ros::spinOnce();

        if(ss=="exit")
        {
            cout<<"Bye bye!"<<endl;
            break;
        }
    }
    return 0;
}
