#ifndef KINEMATICCHAINSNODE
#define KINEMATICCHAINSNODE

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <kinematic_chains/manipulator.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kinematic_chains/CalculateIK.h>
#include <kinematic_chains/ChangeEffectorDescription.h>
#include <kinematic_chains/InitJoints.h>

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
    ros::ServiceServer service1, service2, service3;
    bool ikService(kinematic_chains::CalculateIK::Request &req,
                   kinematic_chains::CalculateIK::Response &res);
    bool changeEffectorService(kinematic_chains::ChangeEffectorDescription::Request &req,
                               kinematic_chains::ChangeEffectorDescription::Response &res);
    bool jointInitializationService(kinematic_chains::InitJoints::Request &req,
                                    kinematic_chains::InitJoints::Response &res);

    KDL::Frame destination;
public:
    KinematicChainsNode(int argc, char **argv);
    void addNewManipulator(KDL::Chain arm, KDL::Chain effector);
    void spin();
    bool chainFromFile(std::string filename, KDL::Chain& outChain);
    bool parseRobotDescription(std::string parameterName, manipulator_part part, const char* root_name, const char* tip_name);

};

#endif
