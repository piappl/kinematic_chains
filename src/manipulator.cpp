#include <kinematic_chains/manipulator.hpp>

using namespace KDL;

Manipulator::Manipulator(Chain arm, Chain efector)
{
    Manipulator();
    this->arm = arm;
    this->effector = efector;
    reinit();
}

Manipulator::Manipulator()
{
    armInitialized = effectorInitialized = false;
}

void Manipulator::reinit()
{
    chain = this->arm;
    chain.addChain(this->effector);

    if(armInitialized) //state of arms' joints has to remain
    {
        JntArray tmp1 = current_q;
        current_q = JntArray(chain.getNrOfJoints());
        for(int i = 0; i<arm.getNrOfJoints();i++)
        {
            current_q(i) = tmp1(i);
        }
    }
    else
    {
        current_q = JntArray(chain.getNrOfJoints());
    }
    previous_q = JntArray(chain.getNrOfJoints());


    fksolver = new ChainFkSolverPos_recursive(chain);
    iksolver = new ChainIkSolverVel_pinv(chain);

    iksolverpos = new ChainIkSolverPos_NR(chain,*fksolver,*iksolver,100,1e-6);

    std::cout<<"manipulator updated, current number of joints " <<chain.getNrOfJoints() <<
               ", arm joints number:" <<arm.getNrOfJoints() <<
               ", tool joints number: "<< effector.getNrOfJoints()<<std::endl;
}

void Manipulator::setEffector(Chain chain)
{
    effectorInitialized = false;
    std::cout<<"initialazing effector"<<std::endl;
    this->effector = chain;
    reinit();
    effectorInitialized = true;
}

void Manipulator::setArm(Chain chain)
{
    armInitialized = false;
    //std::cout<<"initialazing arm"<<std::endl;
    this->arm = chain;
    reinit();
    armInitialized = true;
}


bool Manipulator::calculateIK(Frame destination)
{
    JntArray q_out(chain.getNrOfJoints());

    //ChainFkSolverPos_recursive fksolver(chain);

    //ChainIkSolverVel_pinv iksolver(chain);
    //iksolverpos = new ChainIkSolverPos_NR(chain,fksolver,iksolver,100,1e-6);
    //std::cout<<"calculating inverse kinematics"<<std::endl;
    previous_q = current_q;
    if( ChainIkSolverPos_NR::E_NOERROR == iksolverpos->CartToJnt(previous_q,destination,q_out))
    {
        //ROS_INFO("Solution found");
        current_q = q_out;
        return true;
    }
    //ROS_ERROR("No solution found");
    return false;
}

bool Manipulator::isInitialized()
{
    return armInitialized && effectorInitialized;
}

std::string Manipulator::getJntName(int i)
{
    std::stringstream ss;
    if(i <=arm.getNrOfJoints())
        ss<<"arm_";
    else
        ss<<"effector_";
    ss<<chain.getSegment(i).getJoint().getName();
    return ss.str();
}

bool Manipulator::setJoints(JntArray joints)
{
    if(joints.rows()==current_q.rows())
    {
        current_q = joints;
        return true;
    }else
    {
        return false;
    }
}



