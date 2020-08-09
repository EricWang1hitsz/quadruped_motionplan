#include <ros/ros.h>
#include <MotionPlanningTest.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionPlanningTest");
    motion_planning mopl;
//    mopl.setStart(-2, -2, 1.35); // default start pose.
//    mopl.setGoal(1, 1, 1);
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("PLANNING FINISHED");

}
