#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ompl/base/State.h>

namespace ob = ompl::base;

class MotionPlannerDisplay
{
public:

    MotionPlannerDisplay();

    ~MotionPlannerDisplay();

    void tree_pub(ob::State *state);

private:

    ros::NodeHandle nodehandle_;

    ros::Publisher tree_pub_;

    visualization_msgs::Marker points, line;

    geometry_msgs::Point p;
};
