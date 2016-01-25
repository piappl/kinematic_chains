#include <kinematic_chains/kinematic_chains.hpp>
#include <cstring>

void help()
{
    std::cout<<"Usage:"<<std::endl;
    std::cout<<"rosrun kinematic_chains kinematic_chains [-arm|-tool relevant_parameter_in_parameter_server [-arm_base|-arm_tip|-tool_base|-tool_tip] relevant_link_name_in_urdf_description]"<<std::endl;
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
