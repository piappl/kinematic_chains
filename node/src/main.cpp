#include <node/node.hpp>
#include <cstring>

int main(int argc, char **argv)
{
    for(int i = 1; i< argc; i++)
    {
        if( strcmp(argv[i],"-h") == 0){
            std::cout<<"Help under construction"<<std::endl;
            exit(0);
        }
    }

    KinematicChainsNode node(argc, argv);
    node.spin();

    return 0;
}
