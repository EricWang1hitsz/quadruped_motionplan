#include "MotionPlannerDisplay.hpp"


void MotionPlannerDisplay::tree_pub(ob::State *state)
{
    points.header.frame_id = "odom";
    line.header.frame_id = "odom";

    points.header.stamp = ros::Time(0);
    line.header.stamp = ros::Time(0);

    points.ns = "/";
    line.ns = "/";

    line.id = 1;
    line.type = line.LINE_LIST;

    points.action = points.ADD;
    line.action = line.ADD;

    points.pose.orientation.w = 1.0;
    line.pose.orientation.w = 1.0;
    line.scale.x =  0.03;
    line.scale.y= 0.03;
    points.scale.x=0.3;
    points.scale.y=0.3;

    line.color.r =9.0/255.0;
    line.color.g= 91.0/255.0;
    line.color.b =236.0/255.0;
    points.color.r = 255.0/255.0;
    points.color.g = 0.0/255.0;
    points.color.b = 0.0/255.0;
    points.color.a=1.0;
    line.color.a = 1.0;
    points.lifetime = ros::Duration();
    line.lifetime = ros::Duration();


}
