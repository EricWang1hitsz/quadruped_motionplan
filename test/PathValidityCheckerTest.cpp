#include <PathValidityChecker.hpp>
#include <MotionPlanningTest.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_validity_checker");
    ros::NodeHandle nh_;

    ros::Publisher traversabilityMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("/traversability_estimation/traversability_map", 1);
    ros::Publisher footPrintPathPublisher_ = nh_.advertise<geometry_msgs::PoseArray>("footPrintPath", 1);
    std::string topic =
            "/traversability_estimation/traversability_map";
    std::string path_file =
            "/home/eric/traversability_map.bag";
    GridMap gridMap;
    grid_map_msgs::GridMap message_;
    GridMapRosConverter::loadFromBag(path_file, topic, gridMap);
//    for(GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
//    {
//        const Index index(*iterator);
//        auto& traversability = gridMap.at("traversability", index);
//        if(gridMap.isValid(index, "traversability"))
//            continue; // ignore cells without data in local map.
//        traversability = 0.1;
//    }
    ROS_INFO_ONCE("Created map with size %f x %f m (%i x %i cells).",
      gridMap.getLength().x(), gridMap.getLength().y(),
      gridMap.getSize()(0), gridMap.getSize()(1));

    PathValidityChecker pathValidityChecker_(nh_);
    pathValidityChecker_.setTraversabilityMap(gridMap);

    //geometry_msgs::PoseArray footPrintPath_;
    //footPrintPath todo
    //pathValidityChecker_.setFootPrintPath(footPrintPath_);

    traversability_msgs::FootprintPath footPrintPath_;
    geometry_msgs::PoseArray msgs_;
    msgs_.header.stamp = ros::Time();
    msgs_.header.frame_id = "odom";
    geometry_msgs::Pose msg_1;
    msg_1.position.x = 0.0;
    msg_1.position.y = 0.0;
    msg_1.position.z = 0.5;
    msg_1.orientation.x = 0.0;
    msg_1.orientation.y = 0.0;
    msg_1.orientation.z = 0.0;
    msg_1.orientation.w = 1.0;
    geometry_msgs::Pose msg_2;
    msg_2.position.x = 0.0;
    msg_2.position.y = -1.0;
    msg_2.position.z = 0.5;
    msg_2.orientation.x = 0.0;
    msg_2.orientation.y = 0.0;
    msg_2.orientation.z = 0.0;
    msg_2.orientation.w = 1.0;
    geometry_msgs::Pose msg_3;
    msg_3.position.x = 0.0;
    msg_3.position.y = -2.0;
    msg_3.position.z = 0.5;
    msg_3.orientation.x = 0.0;
    msg_3.orientation.y = 0.0;
    msg_3.orientation.z = 0.0;
    msg_3.orientation.w = 1.0;
    geometry_msgs::Pose msg_4;
    msg_4.position.x = 1.0;
    msg_4.position.y = -2.0;
    msg_4.position.z = 0.5;
    msg_4.orientation.x = 0.0;
    msg_4.orientation.y = 0.0;
    msg_4.orientation.z = 0.0;
    msg_4.orientation.w = 1.0;
    msgs_.poses.push_back(msg_1);
    msgs_.poses.push_back(msg_2);
    msgs_.poses.push_back(msg_3);
    msgs_.poses.push_back(msg_4);
    footPrintPath_.poses = msgs_;
    footPrintPath_.radius = 0.3;
    pathValidityChecker_.setFootPrintPath(footPrintPath_);

    //Initial planner.
    motion_planning motionPlanning_;

    ros::Rate rate_(0.1);

    while(ros::ok())
    {
        pathValidityChecker_.checkFootprintPath();
////        ROS_INFO_THROTTLE(5, "check Footprint Path 5 times");
        GridMapRosConverter::toMessage(gridMap, message_);
        traversabilityMapPublisher_.publish(message_);
        footPrintPathPublisher_.publish(msgs_);
        geometry_msgs::PoseArray segmentsIsUntraversable_;
        segmentsIsUntraversable_ = pathValidityChecker_.getSegmentIsUntraversable();
        motionPlanning_.setSegmentIsUntraversable(segmentsIsUntraversable_);
//        motionPlanning_.setSegmentIsUntraversable(msgs_);
        motionPlanning_.setReplanInitialInfo();
        motionPlanning_.plan();
        ros::spinOnce();
        rate_.sleep();
//        ros::shutdown();
    }

    return 0;

}
