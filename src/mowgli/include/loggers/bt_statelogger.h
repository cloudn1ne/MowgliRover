#ifndef BT_STATELOGGER_H
#define BT_STATELOGGER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <deque>
#include <array>
#include "behaviortree_cpp_v3/loggers/abstract_logger.h"

namespace BT
{
class StateLogger : public StatusChangeLogger
{
public:
  StateLogger(const Tree& tree, ros::Publisher pubCurrentState, uint16_t buffer_size = 10);
    
  virtual ~StateLogger() override;

  virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                        NodeStatus status) override;

  virtual void flush() override;

private:  
  size_t buffer_max_size_;
  ros::Publisher _pubCurrentState;
  
};

}   // namespace BT

#endif   // BT_STATELOGGER_H