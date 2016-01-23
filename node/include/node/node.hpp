#ifndef KINEMATICCHAINSNODE
#define KINEMATICCHAINSNODE

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <node/manipulator.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <node/CalculateIK.h>
#include <node/ChangeEffectorDescription.h>

class KinematicChainsNode
{
private:

    double goal[6];
    ros::NodeHandle *n;
    ros::Subscriber joySubscriber;
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    tf::TransformBroadcaster *br;
    Manipulator *manipulator;
    void publishForwardKinematics();
    void publishGoal();
    ros::ServiceServer service;
    bool ikService(node::CalculateIK::Request &req,
                   node::CalculateIK::Response &res);
    bool changeEffectorService(node::ChangeEffectorDescription::Request &req,
                               node::ChangeEffectorDescription::Response &res);
    KDL::Frame destination;
public:
    KinematicChainsNode(int argc, char **argv);
    void addNewManipulator(KDL::Chain arm, KDL::Chain effector);
    void spin();
    bool chainFromFile(std::string filename, KDL::Chain& outChain);
    bool parseRobotDescription(std::string parameterName, manipulator_part part, const char* root_name, const char* tip_name);
};

#endif
