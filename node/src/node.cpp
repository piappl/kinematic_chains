#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <node/kinematic_chain_handler.hpp>


double x = 0, y = 0, z = 0, X = 0, Y = 0, Z = 1;
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    x = X + msg->axes[0];
    y = Y + msg->axes[1];
    z = Z + msg->axes[2];
    //std::cout<<"joy "<< x<< ", "<<y<<std::endl;
}

KDL::Frame forward(KDL::ChainFkSolverPos_recursive fksolver, KDL::JntArray jnt, KDL::Chain chain, tf::TransformBroadcaster br, std::string prefix="")
{
    KDL::Frame cartpos, return_pos;

    fksolver.JntToCart(jnt, return_pos);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(return_pos.p[0], return_pos.p[1], return_pos.p[2]));
    tf::Quaternion q;
    double r,p,y;
    return_pos.M.GetRPY(r,p,y);
    q.setRPY(r,p,y);
    transform.setRotation(q);
    std::stringstream s1, s2;
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "cart_pos_fk_slover"));

    int j = 0;
    for(int k = 0; k < chain.getNrOfSegments(); k++)
    {
        if(chain.getSegment(k).getJoint().getType()!=KDL::Joint::None){
            cartpos = chain.getSegment(k).pose(jnt(j));
            j++;
        }else{
            cartpos = chain.getSegment(k).pose(0.0);
        }

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(cartpos.p[0], cartpos.p[1], cartpos.p[2]));
        tf::Quaternion q;
        double r,p,y;
        cartpos.M.GetRPY(r,p,y);
        q.setRPY(r,p,y);
        transform.setRotation(q);
        std::stringstream s1, s2;
        s2 <<prefix<< "cart_pos"<<k;
        if(k == 0)
            s1<< "world";
        else
            s1 <<prefix<< "cart_pos"<<k-1;
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), s1.str().c_str(), s2.str().c_str()));
    }
    return return_pos;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node");
    ros::NodeHandle n;
    tf::TransformBroadcaster br;

    ros::Subscriber sub = n.subscribe("joy", 100, joyCallback);

    ros::Rate loop_rate(20);

    KDL::Chain chain2, chain1;
    
    chain2.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
    chain2.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,1))));
    chain2.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1))));
    chain2.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.5))));
    chain2.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.2))));


    KDL::ChainFkSolverPos_recursive fksolver2(chain2);

    Manipulator m = Manipulator(chain2, chain1);


    while (ros::ok())
    {
        //std::cout<<"asd"<<std::endl;

        //****************************************************************************************************//
        //****************************************FORWARD KINEMATICS******************************************//
        //****************************************************************************************************//
        //std::cout<<"joints pos :";
        //for(int i=0; i < chain1.getNrOfJoints(); i++ ) {
        //    jnt(i) +=0.01;
        //    std::cout<< jnt(i) << ", ";
        //}
        //std::cout<<std::endl;
        //
        //KDL::Frame cartpos = forward(fksolver1, jnt, chain1, br);

        KDL::Frame destination;
        destination.p[0] = x;
        destination.p[1] = y;
        destination.p[2] = z;

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(destination.p[0], destination.p[1], destination.p[2]));
        tf::Quaternion quat;
        double R,P,Y;
        destination.M.GetRPY(R,P,Y);
        quat.setRPY(R,P,Y);
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));



        //****************************************************************************************************//
        //*****************************************IVERSE KINEMATICS******************************************//
        //****************************************************************************************************//
        //std::cout<<"asd2"<<std::endl;
        JntArray q = m.calculateIK(destination);
        //std::cout<<"asd3"<<std::endl;
        forward(fksolver2, q, chain2, br, "iksolved_");

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
