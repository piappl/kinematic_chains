#ifndef KINEMATIC_CHAIN_HANDLER
#define KINEMATIC_CHAIN_HANDLER

#include "std_msgs/String.h"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <tf/transform_broadcaster.h>
#include <kdl/chainiksolvervel_pinv.hpp>

using namespace KDL;

enum manipulator_part{
    Arm,
    Effector
};

class Manipulator
{
private:
    Chain arm, effector, chain;
    ChainIkSolverPos_NR* iksolverpos;
    JntArray previous_q, current_q;
    ChainFkSolverPos_recursive* fksolver;
    ChainIkSolverVel_pinv* iksolver;
    bool armInitialized, effectorInitialized;
    void reinit();

public:
    Manipulator(Chain arm, Chain efector);
    Manipulator();
    void setEffector(Chain chain);
    void setArm(Chain chain);
    bool calculateIK(Frame destination);
    Frame calculateFK(JntArray joints);
    Chain getChain(){return chain;}
    JntArray getJnts(){return current_q;}
    bool isInitialized();
    bool isArmInitialized(){return armInitialized;}
    std::string getJntName(int i);


};

#endif
