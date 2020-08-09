#include <PathValidityChecker.hpp>
//todo reset function
PathValidityChecker::PathValidityChecker(ros::NodeHandle& nodeHandle)
    : nodehandle_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      checkForRoughness_(false),
      maxGapWidth_(0.3),
      criticalStepHeight_(0.02)

{
    positionIsUntraversablePublisher_ = nodehandle_.advertise<visualization_msgs::Marker>("positionIsUntraversable", 1);
    segmentIsUntraversablePublisher_ = nodehandle_.advertise<visualization_msgs::Marker>("segmentIsUntraversable", 1);
    ROS_INFO("Path Validity Checker Initialized");
}

void PathValidityChecker::footPrintPathCallback(const geometry_msgs::PoseArrayPtr &footPrintPath)
{
    geometry_msgs::PoseArray poseArray_ = *footPrintPath;
    footPrintPath_ = poseArray_;
    ROS_INFO("Get footPrint Path Message");
}

bool PathValidityChecker::isTraversable(const grid_map::Position &center, const double &radiusMax, const bool &computeUntraversablePolygon,
                                        double &traversability, grid_map::Polygon &untraversablePolygon, const double &radiusMin)
{
    //ROS_INFO("Check footprint traversability for each cell");
    bool circleIsTraversable = true;
    std::vector<grid_map::Position> untraversablePositions;
    grid_map::Position positionUntraversableCell;
    untraversablePolygon = grid_map::Polygon();  // empty untraversable polygon
    // Handle cases of footprints outside of map.
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    if (!traversabilityMap_.isInside(center)) {
      //1 Footprints outside map.
      traversability = traversabilityDefault_;
      circleIsTraversable = traversabilityDefault_ != 0.0;
      if (computeUntraversablePolygon && !circleIsTraversable) {
        //!Eric_Wang: Add vertex to polygon.
        untraversablePolygon = grid_map::Polygon::fromCircle(center, radiusMax);
      }
    } else {
      //2 Footprints inside map.
      // Get index of center position.
      grid_map::Index indexCenter;
      traversabilityMap_.getIndex(center, indexCenter);
      if (traversabilityMap_.isValid(indexCenter, "traversability_footprint")) {
        traversability = traversabilityMap_.at("traversability_footprint", indexCenter);
        circleIsTraversable = traversability != 0.0;
        if (computeUntraversablePolygon && !circleIsTraversable) {
          untraversablePolygon = grid_map::Polygon::fromCircle(center, radiusMax);
        }
      } else {
        //ROS_WARN("Non valid in traversability footprint");
        // Non valid (non finite traversability)
        int nCells = 0;
        traversability = 0.0;

        // Iterate through polygon and check for traversability.
        double maxUntraversableRadius = 0.0;
        bool traversableRadiusBiggerMinRadius = false;
        for (grid_map::SpiralIterator iterator(traversabilityMap_, center, radiusMax);
             !iterator.isPastEnd() && !traversableRadiusBiggerMinRadius; ++iterator) {
          //!Eric_Wang: Check traversability for slope, step and roughness.
          const bool currentPositionIsTraversale = isTraversableForFilters(*iterator);
          if (!currentPositionIsTraversale) {
            //!TODO output position in rviz for visulization.
            traversabilityMap_.getPosition(*iterator, positionIsUntraversable);
            publishPositionUntraversable();
            //ROS_INFO_STREAM("position is untraversable: " << positionIsUntraversable[0] << positionIsUntraversable[1] << std::endl);
            ROS_WARN("Current position is untraversale when spiraliterator");
            // current position is untraversale.
            const auto untraversableRadius = iterator.getCurrentRadius();
            maxUntraversableRadius = std::max(maxUntraversableRadius, untraversableRadius);

            if (radiusMin == 0.0) {
              traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
              circleIsTraversable = false;
              traversabilityMap_.getPosition(*iterator, positionUntraversableCell);
              untraversablePositions.push_back(positionUntraversableCell);
            } else {
              if (untraversableRadius <= radiusMin) {
                traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
                circleIsTraversable = false;
                traversabilityMap_.getPosition(*iterator, positionUntraversableCell);
                untraversablePositions.push_back(positionUntraversableCell);
              } else if (circleIsTraversable) {  // if circleIsTraversable is not changed by any previous loop
                auto factor = ((untraversableRadius - radiusMin) / (radiusMax - radiusMin) + 1.0) / 2.0;
                traversability *= factor / nCells;
                traversabilityMap_.at("traversability_footprint", indexCenter) = static_cast<float>(traversability);
                circleIsTraversable = true;
                traversableRadiusBiggerMinRadius = true;
              }
            }

            if (!computeUntraversablePolygon) {
              // Do not keep on checking, one cell is already non-traversable.
              return false;
            }
          } else { // current postion is traversale.
            ROS_WARN_ONCE("Current position is traversale when spiraliterator");
            nCells++;
            if (!traversabilityMap_.isValid(*iterator, "traversability")) {
              traversability += traversabilityDefault_;
            } else {// traversability map is valid in the index.
              traversability += traversabilityMap_.at("traversability", *iterator);
            }
          }
        } // Iterate ends.

        if (computeUntraversablePolygon && !circleIsTraversable) {
          untraversablePolygon = grid_map::Polygon::monotoneChainConvexHullOfPoints(untraversablePositions);
        }

        if (circleIsTraversable) {
          traversability /= nCells;
          //eric_wang: Add footprint traversability into the traversability_footprint layer.
          traversabilityMap_.at("traversability_footprint", indexCenter) = static_cast<float>(traversability);
          //ROS_INFO_STREAM("traversability footprint is " << traversabilityMap_.at("traversability_footprint", indexCenter) << std::endl);
        }
      }
    //!Eric_Wang: Non valid (non finite traversability)
    }
    scopedLockForTraversabilityMap.unlock();

//    if (computeUntraversablePolygon) {
//      untraversablePolygon.setFrameId(getMapFrameId());
//      untraversablePolygon.setTimestamp(ros::Time::now().toNSec());
//    }

    return circleIsTraversable;
}

bool PathValidityChecker::checkCircularFootprintPath(const traversability_msgs::FootprintPath &path, const bool publishPolygons,
                                                     traversability_msgs::TraversabilityResult &result)
{   ROS_INFO_ONCE("Check Circular Footprint Path");
    double radius = path.radius;
    double offset = 0.15;
    grid_map::Position start, end;
    const auto arraySize = path.poses.poses.size();
    ROS_INFO_STREAM("Segment Number is: " << arraySize << std::endl);
    const bool computeUntraversablePolygon = path.compute_untraversable_polygon;
    result.is_safe = static_cast<unsigned char>(false);
    //!Eric_Wang: Return 0 if the footprint traversability of one cell is untraversable.
    result.traversability = 0.0;
    result.area = 0.0;
    double traversability = 0.0;
    double area = 0.0;
    grid_map::Polygon untraversablePolygon;
    // Cal robot height info.
//    auto robotHeight = computeMeanHeightFromPoses(path.poses.poses);

    bool pathIsTraversable = true; // the whole path is traversable or not.
    for (int i = 0; i < arraySize; i++) {
      ROS_WARN_STREAM("Segment " << i << " is checked " << std::endl);
      start = end; // first start is poses[0].
      end.x() = path.poses.poses[i].position.x;
      end.y() = path.poses.poses[i].position.y;

      if (arraySize == 1) {
//        if (checkRobotInclination_) {
//          if (!checkInclination(end, end)) {
//            return true;
//          }
//        }
        bool pathIsTraversable =
            isTraversable(end, radius + offset, computeUntraversablePolygon, traversability, untraversablePolygon, radius);
//        if (publishPolygons) {
//          grid_map::Polygon polygon = grid_map::Polygon::fromCircle(end, radius + offset);
//          polygon.setFrameId(getMapFrameId());
//          polygon.setTimestamp(ros::Time::now().toNSec());
//          publishFootprintPolygon(polygon);
//          if (computeUntraversablePolygon) {
//            publishUntraversablePolygon(untraversablePolygon, robotHeight);
//          }
//        }
        if (!pathIsTraversable) {
          // return such that default values in result - i.e. non traversable - are used.
          return true;
        }
        result.traversability = traversability;
      }

      if (arraySize > 1 && i > 0) {
//        if (checkRobotInclination_) {
//          if (!checkInclination(start, end)) {
//            return true;
//          }
//        }
        double traversabilityTemp, traversabilitySum = 0.0;
        int nLine = 0;
        grid_map::Index startIndex, endIndex;
        boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
        traversabilityMap_.getIndex(start, startIndex);
        traversabilityMap_.getIndex(end, endIndex);
        int nSkip = 3;  // TODO: Remove magic number.
        grid_map::Polygon auxiliaryUntraversablePolygon;
//        bool pathIsTraversable = true; // the whole path is traversable or not.
        bool segmentIsTraversable = true; // the segment is traversable or not.
        //!eric_wang: Line iterator between two adjacent poses.
        ROS_INFO("test");
        for (grid_map::LineIterator lineIterator(traversabilityMap_, endIndex, startIndex); !lineIterator.isPastEnd(); ++lineIterator)
        {
          bool footprintIsTraversable = true; // one footprint is traversable or not.
          grid_map::Position center;
          traversabilityMap_.getPosition(*lineIterator, center);
          //!Eric_Wang: pathIsTraversable return false only when return false once.
//          pathIsTraversable = pathIsTraversable && isTraversable(center, radius + offset, computeUntraversablePolygon, traversabilityTemp,
//                                                                 auxiliaryUntraversablePolygon, radius);
          segmentIsTraversable = segmentIsTraversable && isTraversable(center, radius + offset, computeUntraversablePolygon, traversabilityTemp,
                                                                   auxiliaryUntraversablePolygon, radius);

          if (publishPolygons && computeUntraversablePolygon && auxiliaryUntraversablePolygon.nVertices() > 0) {
            untraversablePolygon = grid_map::Polygon::convexHull(untraversablePolygon, auxiliaryUntraversablePolygon);
          }

//          if (!pathIsTraversable && !computeUntraversablePolygon && !publishPolygons) {
//            // return such that default values in result - i.e. non traversable - are used.
//            return true;
//          }

          traversabilitySum += traversabilityTemp;
          nLine++;
          for (int j = 0; j < nSkip; j++) {
            if (!lineIterator.isPastEnd()) {
              ++lineIterator;
            }
          }
        }
        ROS_INFO("test1");
        if(!segmentIsTraversable)
        {
            //!Eric_Wang: Add untraversable segment.
            ROS_INFO("Add one untraversable segment once");
            if(pathIsUntraversable_.poses.empty())
            {
                //!Eric_Wang: First segment add start and end point.
                pathIsUntraversable_.poses.push_back(path.poses.poses[i - 1]);// untraversable segment start.
                pathIsUntraversable_.poses.push_back(path.poses.poses[i]); // untraversable segement end.
            }
            else {
                pathIsUntraversable_.poses.push_back(path.poses.poses[i]);
            }

//            publishSegmentUntraversable();
//            pathIsUntraversable_.poses.clear();
        }
        //!Eric_Wang: Path return false if has one untraversable segment.
        pathIsTraversable = pathIsTraversable && segmentIsTraversable;
        scopedLockForTraversabilityMap.unlock();

//        if (publishPolygons) {
//          grid_map::Polygon polygon = grid_map::Polygon::fromCircle(end, radius + offset);
//          polygon.setFrameId(getMapFrameId());
//          polygon.setTimestamp(ros::Time::now().toNSec());
//          publishFootprintPolygon(polygon);
//          if (computeUntraversablePolygon) {
//            untraversablePolygon.setFrameId(auxiliaryUntraversablePolygon.getFrameId());
//            untraversablePolygon.setTimestamp(auxiliaryUntraversablePolygon.getTimestamp());
//            publishUntraversablePolygon(untraversablePolygon, robotHeight);
//          }
//        }

//        if (pathIsTraversable) {
        if(segmentIsTraversable){
          //!Eric_Wang: Current segment traversability.
          traversability = traversabilitySum / (double)nLine;
          ROS_INFO_STREAM("Current segment traversability " << traversability << std::endl);
          double lengthSegment, lengthPreviousPath, lengthPath;
          lengthSegment = (end - start).norm();
          if (i > 1) {
            lengthPreviousPath = lengthPath;
            lengthPath += lengthSegment;
            //!Eric_Wang: whole path traversability.
            result.traversability = (lengthSegment * traversability + lengthPreviousPath * result.traversability) / lengthPath;
          } else {
            lengthPath = lengthSegment;
            result.traversability = traversability; // the whole path traversability.
          }
        }
//        else {
            //!Eric_Wang: Stop loop once one segment is untraversable.
//          // return such that default values in result - i.e. non traversable - are used.
//          return true;
//        }
      }
    }

    //result.is_safe = static_cast<unsigned char>(true);
    result.is_safe = static_cast<unsigned char>(pathIsTraversable);
    return true;
}

bool PathValidityChecker::isTraversableForFilters(const grid_map::Index &indexStep)
{
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    bool currentPositionIsTraversale = true;
    if (checkForSlope(indexStep)) {
      if (checkForStep(indexStep)) {
        if (checkForRoughness_) {
          if (!checkForRoughness(indexStep)) {
            currentPositionIsTraversale = false;
          }
        }
      } else {
        currentPositionIsTraversale = false;
      }
    } else {
      currentPositionIsTraversale = false;
    }

    return currentPositionIsTraversale;
}

bool PathValidityChecker::checkForStep(const grid_map::Index &indexStep)
{
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    if (traversabilityMap_.at(stepType_, indexStep) == 0.0) {
      if (!traversabilityMap_.isValid(indexStep, "step_footprint")) {
        double windowRadiusStep = 2.5 * traversabilityMap_.getResolution();  // 0.075;

        std::vector<grid_map::Index> indices;
        grid_map::Position center;
        traversabilityMap_.getPosition(indexStep, center);
        double height = traversabilityMap_.at("elevation", indexStep);
        for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadiusStep); !circleIterator.isPastEnd();
             ++circleIterator) {
          if (traversabilityMap_.at("elevation", *circleIterator) > criticalStepHeight_ + height &&
              traversabilityMap_.at(stepType_, *circleIterator) == 0.0)
            indices.push_back(*circleIterator);
        }
        if (indices.empty()) indices.push_back(indexStep);
        for (auto& index : indices) {
          grid_map::Length subMapLength(2.5 * traversabilityMap_.getResolution(), 2.5 * traversabilityMap_.getResolution());
          grid_map::Position subMapPos;
          bool isSuccess;
          traversabilityMap_.getPosition(index, subMapPos);
          grid_map::Vector toCenter = center - subMapPos;
          grid_map::GridMap subMap = traversabilityMap_.getSubmap(subMapPos, subMapLength, isSuccess);
          if (!isSuccess) {
            ROS_WARN("Traversability map: Check for step window could not retrieve submap.");
            traversabilityMap_.at("step_footprint", indexStep) = 0.0;
            return false;
          }
          height = traversabilityMap_.at("elevation", index);
          for (grid_map::GridMapIterator subMapIterator(subMap); !subMapIterator.isPastEnd(); ++subMapIterator) {
            if (subMap.at(stepType_, *subMapIterator) == 0.0 && subMap.at("elevation", *subMapIterator) < height - criticalStepHeight_) {
              grid_map::Position pos;
              subMap.getPosition(*subMapIterator, pos);
              grid_map::Vector vec = pos - subMapPos;
              if (vec.norm() < 0.025) continue;
              if (toCenter.norm() > 0.025) {
                if (toCenter.dot(vec) < 0.0) continue;
              }
              pos = subMapPos + vec;
              while ((pos - subMapPos + vec).norm() < maxGapWidth_ && traversabilityMap_.isInside(pos + vec)) pos += vec;
              grid_map::Index endIndex;
              traversabilityMap_.getIndex(pos, endIndex);
              bool gapStart = false;
              bool gapEnd = false;
              for (grid_map::LineIterator lineIterator(traversabilityMap_, index, endIndex); !lineIterator.isPastEnd(); ++lineIterator) {
                if (traversabilityMap_.at("elevation", *lineIterator) > height + criticalStepHeight_) {
                  traversabilityMap_.at("step_footprint", indexStep) = 0.0;
                  return false;
                }
                if (traversabilityMap_.at("elevation", *lineIterator) < height - criticalStepHeight_ ||
                    !traversabilityMap_.isValid(*lineIterator, "elevation")) {
                  gapStart = true;
                } else if (gapStart) {
                  gapEnd = true;
                  break;
                }
              }
              if (gapStart && !gapEnd) {
                traversabilityMap_.at("step_footprint", indexStep) = 0.0;
                return false;
              }
            }
          }
        }
        traversabilityMap_.at("step_footprint", indexStep) = 1.0;
      } else if (traversabilityMap_.at("step_footprint", indexStep) == 0.0) {
        return false;
      }
    }
    return true;
}

bool PathValidityChecker::checkForSlope(const grid_map::Index &index)
{
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    if (traversabilityMap_.at(slopeType_, index) == 0.0) {
      if (!traversabilityMap_.isValid(index, "slope_footprint")) {
        double windowRadius = 3.0 * traversabilityMap_.getResolution();  // TODO: read this as a parameter?
        double criticalLength = maxGapWidth_ / 3.0;
        int nSlopesCritical = floor(2 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

        // Requested position (center) of circle in map.
        grid_map::Position center;
        traversabilityMap_.getPosition(index, center);
        int nSlopes = 0;
        for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius); !circleIterator.isPastEnd();
             ++circleIterator) {
          if (traversabilityMap_.at(slopeType_, *circleIterator) == 0.0) nSlopes++;
          if (nSlopes > nSlopesCritical) {
            traversabilityMap_.at("slope_footprint", index) = 0.0;
            return false;
          }
        }
        traversabilityMap_.at("slope_footprint", index) = 1.0;
      } else if (traversabilityMap_.at("slope_footprint", index) == 0.0) {
        return false;
      }
    }
    return true;
}

bool PathValidityChecker::checkForRoughness(const grid_map::Index &index)
{
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    if (traversabilityMap_.at(roughnessType_, index) == 0.0) {
      if (!traversabilityMap_.isValid(index, "roughness_footprint")) {
        double windowRadius = 3.0 * traversabilityMap_.getResolution();  // TODO: read this as a parameter?
        double criticalLength = maxGapWidth_ / 3.0;
        int nRoughnessCritical = floor(1.5 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

        // Requested position (center) of circle in map.
        grid_map::Position center;
        traversabilityMap_.getPosition(index, center);
        int nRoughness = 0;
        for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius); !circleIterator.isPastEnd();
             ++circleIterator) {
          if (traversabilityMap_.at(roughnessType_, *circleIterator) == 0.0) nRoughness++;
          if (nRoughness > nRoughnessCritical) {
            traversabilityMap_.at("roughness_footprint", index) = 0.0;
            return false;
          }
        }
        traversabilityMap_.at("roughness_footprint", index) = 1.0;
      } else if (traversabilityMap_.at("roughness_footprint", index) == 0.0) {
        return false;
      }
    }
    return true;
}

void PathValidityChecker::setDefaultTraversabilityUnknownRegions(const double &defaultTraversability)
{
    traversabilityDefault_ = defaultTraversability;
}

void PathValidityChecker::setTraversabilityMap(const grid_map::GridMap gridMap)
{
    traversabilityMap_ = gridMap;
    ROS_INFO("Receive traversability Map");
}

void PathValidityChecker::setFootPrintPath(const geometry_msgs::PoseArray footPrintPath)
{
    footPrintPath_ = footPrintPath;
    ROS_INFO("Receive footPrint Path");
}

void PathValidityChecker::setFootPrintPath(const traversability_msgs::FootprintPath footPrintPathMsg)
{
    footPrintPathMsg_ = footPrintPathMsg;
    ROS_INFO("Receive Traversability FootPrint Path");
}

void PathValidityChecker::checkFootprintPath()
{
    bool publishPolygons_ = false;
    checkCircularFootprintPath(footPrintPathMsg_, publishPolygons_, result_);
    bool safe = result_.is_safe;
    if(safe)
        ROS_WARN("Whole Footprint Path is Safe");
    else {
        ROS_WARN("Whole Footprint Path is unsafe");
    }
    ROS_INFO_STREAM("Path traversability is " << result_.traversability << std::endl);
    ROS_INFO_STREAM("Untraversable Segment number " <<pathIsUntraversable_.poses.size() << std::endl);
    publishSegmentUntraversable();
//    pathIsUntraversable_.poses.clear();
}

void PathValidityChecker::publishPositionUntraversable()
{
    ROS_INFO("Publish Marker Untraversable");
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = positionIsUntraversable[0];
    marker.pose.position.y = positionIsUntraversable[1];
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    positionIsUntraversablePublisher_.publish(marker);

}

void PathValidityChecker::publishSegmentUntraversable()
{
    ROS_INFO("Publish Segment Untraversable");
    geometry_msgs::PoseArray path = pathIsUntraversable_;
    const auto arraySize = path.poses.size();

    visualization_msgs::Marker lines_;
    lines_.header.stamp = ros::Time();
    lines_.header.frame_id = "odom";
    lines_.type = visualization_msgs::Marker::LINE_STRIP;
    lines_.type = visualization_msgs::Marker::ADD;
    lines_.scale.x = 0.04;
    lines_.scale.y = 0.04;
    lines_.scale.z = 0.04;
    lines_.color.a = 1.0;
    lines_.color.r = 1.0;
    lines_.color.g = 0.0;
    lines_.color.b = 0.0;
    for(int i = 0; i < arraySize; i++)
    {
        geometry_msgs::Point point;
        point.x = path.poses[i].position.x;
        point.y = path.poses[i].position.y;
        //ROS_INFO_STREAM("Point y axis " << point.y << std::endl);
        point.z = 0.5; // default
        lines_.points.push_back(point);
        //ROS_INFO_STREAM("point num in line: " << lines_.points.size() << std::endl);
    }
    segmentIsUntraversablePublisher_.publish(lines_);
}

geometry_msgs::PoseArray PathValidityChecker::getSegmentIsUntraversable()
{
    return pathIsUntraversable_;
}
