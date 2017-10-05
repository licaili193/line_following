#include "ros/ros.h"
#include "std_msgs/String.h"

#include "line_following/MergeControl.h"
#include "Merge_builder.h"
#include "Control_zone.h"

#include <vector>
#include <chrono>

bool isRun = true;

MergeBuilder theMerge;

double GetTime()
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Heard command: [%s]", msg->data.c_str());
    if(strcmp(msg->data.c_str(),"exit")==0) isRun = false;
}

bool add(line_following::MergeControl::Request  &req,
         line_following::MergeControl::Response &res)
{
    int id = (int)req.id;
    int control_id = (int)req.cid;
    short control_type = (short)req.ctype;
    short action_type = (short)req.atype;
    double speed = (double)req.speed;

    if(control_id>=theMerge.zoneList.size()||control_type>=theMerge.zoneList[control_id]->entrences.size())
    {
        res.res = 1;
        ROS_INFO("Failed to give result. Error code: 1 (control zone unregistered)");
    }
    else
    {
        EntrenceInfo control_length = theMerge.zoneList[control_id]->entrences[control_type];
        if(action_type==0)//insert
        {
            res.res = 0;
            if(theMerge.zoneList[control_id]->q.empty())
            {
                ControlInfo info = {
                    id,
                    control_type,
                    speed,
                    GetTime(),
                    GetTime()+control_length.L/speed,
                    GetTime()+(control_length.L+control_length.S)/speed
                };
                theMerge.zoneList[control_id]->init_time=GetTime();
                theMerge.zoneList[control_id]->q.push(info);
                res.isFirst = 1;
                ROS_INFO("Robot %d entered the control zone %d. No front robot.", id, control_id);
            }
            else
            {
                ControlInfo pin = theMerge.zoneList[control_id]->q.back();
                int front_id = pin.robot_id;
                ControlInfo info;
                if(pin.entrence_id==control_type) 
                info = {
                    id,
                    control_type,
                    speed,
                    GetTime(),
                    pin.tf,
                    pin.tf+control_length.delta/speed
                };
                else
                info = {
                    id,
                    control_type,
                    speed,
                    GetTime(),
                    pin.tf,
                    pin.tf+control_length.S/speed
                };
                theMerge.zoneList[control_id]->q.push(info);
                res.isFirst = 0;
                res.tinit = theMerge.zoneList[control_id]->init_time;
                res.tmi = info.tm;
                res.L = control_length.L;
                ROS_INFO("Robot %d entered the control zone %d. Front robot: %d.", id, control_id, front_id);
            }
            
        }
        else
        {
            res.res = 0;
            theMerge.zoneList[control_id]->q.pop();
            int rmc = theMerge.zoneList[control_id]->q.size();
            ROS_INFO("Robot %d left the control zone %d. Remain robots: %d.", id, control_id, rmc);
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    theMerge.BuildMerge();

    ros::init(argc, argv, "lf_merge");

    ros::NodeHandle n;

    ros::MultiThreadedSpinner spinner(10); // Use 4 threads

    ros::ServiceServer service = n.advertiseService("lf_merge", add);
    ROS_INFO("Ready to merging control.");
    
    ros::Subscriber sub = n.subscribe("lf_cmd", 1000, cmdCallback);
    ROS_INFO("Welcome to line_following merging control center!");

    //ros::Rate r(10); // 10 hz

    //while(isRun)
    //{
    //    ros::spinOnce();
    //    r.sleep();
    //}

    spinner.spin();

    return 0;
}
