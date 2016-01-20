#include <node/kinematic_chain_handler.hpp>

using namespace KDL;

Manipulator::Manipulator(Chain arm, Chain efector)
{
    this->arm = arm;
    this->effector = efector;
    reinit();
}

void Manipulator::reinit()
{
    chain = this->arm;
    chain.addChain(this->effector);

    q_init = JntArray(chain.getNrOfJoints());
    q = JntArray(chain.getNrOfJoints());
    std::cout<<"chain, nr of joints " <<chain.getNrOfJoints() <<std::endl;
    ChainFkSolverPos_recursive fksolver(chain);

    ChainIkSolverVel_pinv iksolver(chain);
    iksolverpos = new ChainIkSolverPos_NR(chain,fksolver,iksolver,100,1e-6);
    iksolverpos->CartToJnt(q_init,Frame(),q);
}

void Manipulator::changeEffector(Chain effector)
{
    this->effector = effector;
    reinit();
}

JntArray Manipulator::calculateIK(Frame destination)
{
    //ChainFkSolverPos_recursive fksolver(chain);

    //ChainIkSolverVel_pinv iksolver(chain);
    //iksolverpos = new ChainIkSolverPos_NR(chain,fksolver,iksolver,100,1e-6);

    iksolverpos->CartToJnt(q_init,Frame(),q);
    q_init = q;

    return q;
}

