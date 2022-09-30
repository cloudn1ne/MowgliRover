/*
 * Mowgli GetMowPlan Node V1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 * v1.0: inital release
 *
 */
#include "bt_getmowplan.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "boost/algorithm/string/join.hpp"

#define BT_DEBUG 1

/// @brief return current battery voltage
/// @return float "out" port
BT::NodeStatus GetMowPlan::tick()
{    
#ifdef BT_DEBUG    
      ROS_INFO("mowgli_bt: GetMowPlan::tick()");
#endif    
      int area_index = -1;
      getInput("area_index", area_index);

#ifdef BT_DEBUG    
      ROS_INFO_STREAM("mowgli_bt: GetMowPlan() area_index = " << area_index);
#endif  

      mower_map::GetMowingAreaSrv MowingArea;
      MowingArea.request.index = area_index;
      if (!_svcMapClient.call(MowingArea)) {
#ifdef BT_DEBUG      
            ROS_ERROR_STREAM("mowgli_bt: GetMowPlan(): Error loading mowing area");
#endif        
            return BT::NodeStatus::FAILURE;
      }

      // Area orientation is the same as the first point
      double angle = 0;
      auto points = MowingArea.response.area.area.points;
      if (points.size() >= 2) 
      {
            tf2::Vector3 first(points[0].x, points[0].y, 0);
            for(auto point : points) 
            {
                  tf2::Vector3 second(point.x, point.y, 0);
                  auto diff = second - first;
                  if(diff.length() > 2.0) 
                  {     // we have found a point that has a distance of > 2 m, calculate the angle                        
                        angle = atan2(diff.y(), diff.x());
#ifdef BT_DEBUG                
                        ROS_INFO_STREAM("mowgli_bt: GetMowPlan(): Detected mow angle: " << angle*180/M_PI);
#endif                
                        break;
                  }
            }
      }

      // calculate coverage
      slic3r_coverage_planner::PlanPath planPath;
      planPath.request.angle = angle;
      planPath.request.outline_count = 1; // config.outline_count;
      planPath.request.outline = MowingArea.response.area.area;
      planPath.request.holes = MowingArea.response.area.obstacles;
      planPath.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
      planPath.request.outer_offset = 0.25; // config.outline_offset;
      planPath.request.distance = 0.18; // config.tool_width;

      if (!_svcPlannerGetProgressClient.call(planPath)) 
      {
            ROS_ERROR_STREAM("mowgli_bt: GetMowPlan(): Error during coverage planning");
            return BT::NodeStatus::FAILURE;
      }

   //   std::vector<slic3r_coverage_planner::Path> currentMowingPaths;
      // serialize planPath.response.paths response
      /*
      uint32_t serial_size = ros::serialization::serializationLength(planPath.response.paths);
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
      ros::serialization::OStream stream(buffer.get(), serial_size);
      ros::serialization::serialize(stream, planPath.response.paths);

      std::vector<std::string> list;
      list.push_back("Hello");
      list.push_back("World!");


      std::string str = boost::algorithm::join(list, ";");

      ROS_INFO_STREAM("mowgli_bt: GetMowPlay(): serialized response, size = " << serial_size );
*/
    //  currentMowingPaths = planPath.response.paths;   // array of nav_msgs/Path

      setOutput("mowplan", planPath.response.paths );    
      return BT::NodeStatus::SUCCESS;
}
