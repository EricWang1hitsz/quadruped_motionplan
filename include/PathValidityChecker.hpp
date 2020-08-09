/********************************************************

@File PathValidityChecker.hpp

@Description Check path is valid or not.

@Author  Eric Wang

@Date:   2020-08-1

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/
#pragma once

#include <traversability_msgs/FootprintPath.h>
#include <traversability_msgs/TraversabilityResult.h>
#include <traversability_estimation/TraversabilityMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class PathValidityChecker
{
public:

    /*!
     * \brief PathValidityChecker
     */
//    PathValidityChecker(std::shared_ptr<grid_map::GridMap> traversabilityMapPtr);
    PathValidityChecker(ros::NodeHandle& nodeHandle);
    /*!
     * Sets the default traversability value of unknown regions in the map.
     * @param[in] defaultTraversability new default traversability value of unknown regions in the map
     */
    void setDefaultTraversabilityUnknownRegions(const double& defaultTraversability);
    /*!
     * Gets the traversability value of a circular footprint.
     * @param[in] center the center position of the footprint.
     * @param[in] radiusMax the maximum radius of the footprint.
     * @param[in] computeUntraversablePolygon true if untraversable polygon within submap checked for traversability should be computed.
     * @param[out] traversability traversability value of the footprint.
     * @param[out] untraversablePolygon untraversable polygon within area checked for traversability.
     * @param[in] radiusMin if set (not zero), footprint inflation is applied and radiusMin is the minimum
     * valid radius of the footprint.
     * @return true if the circular footprint is traversable, false otherwise.
     */
    bool isTraversable(const grid_map::Position& center, const double& radiusMax, const bool& computeUntraversablePolygon,
                       double& traversability, grid_map::Polygon& untraversablePolygon, const double& radiusMin = 0);
    /*!
     * Checks the traversability of a circular footprint path and returns the traversability.
     * @param[in] path the footprint path that has to be checked.
     * @param[in] publishPolygons says if checked polygon and untraversable polygon should be computed and published.
     * @param[out] result the traversability result.
     * @return true if successful.
     */
    bool checkCircularFootprintPath(const traversability_msgs::FootprintPath& path, const bool publishPolygons,
                                    traversability_msgs::TraversabilityResult& result);
    /*!
     * Checks if the map is traversable, according to defined filters.
     * @param[in] index index of the map to check.
     * @return true if traversable for defined filters.
     */
    bool isTraversableForFilters(const grid_map::Index& index);
    /*!
     * Checks if the map is traversable, only regarding slope, at the position defined
     * by the map index.
     * Small local slopes are not detected as slopes.
     * @param[in] index index of the map to check.
     * @return true if traversable regarding slope, false otherwise.
     */
    bool checkForSlope(const grid_map::Index& index);
    /*!
     * Checks if the map is traversable, only regarding roughness, at the position defined
     * by the map index.
     * Small local roughness is still detected as traversable terrain.
     * @param[in] index index of the map to check.
     * @return true if traversable regarding roughness, false otherwise.
     */
    bool checkForRoughness(const grid_map::Index& index);
    /*!
     * Checks if the map is traversable, only regarding steps, at the position defined
     * by the map index.
     * Small ditches and holes are not detected as steps.
     * @param[in] index index of the map to check.
     * @return true if no step is detected, false otherwise.
     */
    bool checkForStep(const grid_map::Index& indexStep);

    void footPrintPathCallback(const geometry_msgs::PoseArrayPtr& footPrintPath);

    void setTraversabilityMap(const grid_map::GridMap gridMap);

    void setFootPrintPath(const geometry_msgs::PoseArray footPrintPath);

    void setFootPrintPath(const traversability_msgs::FootprintPath footPrintPathMsg);

    void checkFootprintPath();
    /*!
     * \brief publishPositionUntraversable Publish marker where is untraversable.
     */
    void publishPositionUntraversable();
    /*!
     * \brief publishSegmentUntraversable Publish untraversable segment with red line.
     */
    void publishSegmentUntraversable();

    geometry_msgs::PoseArray getSegmentIsUntraversable();

private:

    ros::NodeHandle nodehandle_;

    ros::Subscriber footPrintPathSubcriber_;
    /*!
     * \brief positionIsUntraversablePublisher_
     * Publish position untraversable when spiral iterator.
     */
    ros::Publisher positionIsUntraversablePublisher_;
    /*!
     * \brief segmentIsUntraversablePublisher_
     * Publish segment untraversable with red line marker.
     */
    ros::Publisher segmentIsUntraversablePublisher_;

    //! Map for cal traversability.
    grid_map::GridMap traversabilityMap_;

    //! Default value for traversability of unknown regions.
    double traversabilityDefault_;

    geometry_msgs::PoseArray footPrintPath_;

    geometry_msgs::PoseArray pathIsUntraversable_;

    traversability_msgs::FootprintPath footPrintPathMsg_;

    traversability_msgs::TraversabilityResult result_;

    boost::recursive_mutex traversabilityMapMutex_;

    //! Robot parameter
    double maxGapWidth_;
    double circularFootprintOffset_;  // TODO: get this with FootprintPath msg.
    double criticalStepHeight_;

    //! Traversability map types.
    const std::string traversabilityType_;
    const std::string slopeType_;
    const std::string stepType_;
    const std::string roughnessType_;
    const std::string robotSlopeType_;

    //! Verify footprint for roughness.
    bool checkForRoughness_;

    //! Position is untraversable.
    grid_map::Position positionIsUntraversable;





};
