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

//    KDL::Chain arm, effector;
//    std::cout<<"chain generation"<<std::endl;
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    effector.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    effector.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    effector.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    effector.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    effector.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));
//    effector.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));

    KinematicChainsNode node(argc, argv);
    //node.addNewManipulator(arm, effector);
    node.spin();

    return 0;
}
