#include <kinematic_chains/kinematic_chains.hpp>
#include <cstring>

void help()
{
    std::cout<<"Usage:"<<std::endl;
    std::cout<<"rosrun kinematic_chains kinematic_chains -arm parameter_name [-arm_base|-arm_tip link_name] [-tool parametaer_name [-tool_base|-tool_tip link_name]]"<<std::endl;
}

int main(int argc, char **argv)
{
    for(int i = 1; i< argc; i++)
    {
        if( strcmp(argv[i],"-h") == 0){
            help();
            exit(0);
        }
    }

    KinematicChainsNode node(argc, argv);
    node.spin();

    return 0;
}
