/*
 * Mowgli BehaviorTree V 1.0
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 *
 * v1.0: inital release
 *
 * needs: sudo apt-get install ros-noetic-behaviortree-cpp-v3
 * 
 */

#include "behaviortree_cpp_v3/bt_factory.h"

#include <signal.h>
#include "ros/ros.h"

#include "bt_nodes.h"

using namespace BT;


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mowgli_bt");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ROS_INFO_STREAM("mowgli_bt: Starting mowgli_bt");


    BehaviorTreeFactory factory;


    using namespace DummyNodes;
    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;

    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));



    // Load tree from XML
    auto tree = factory.createTreeFromFile("../MowgliRover/src/mowgli/src/tree.xml");
    tree.tickRoot();

    ros::Rate r(0.5); // ROS loop rate
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce(); 
    }
    return 0;
}
