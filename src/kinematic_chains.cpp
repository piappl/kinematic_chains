#include <kinematic_chains/kinematic_chains.hpp>

KinematicChainsNode::KinematicChainsNode(int argc, char **argv)
{
    //ROS_INFO("Node init");
    ros::init(argc, argv, "KinematicChains");
    n = new ros::NodeHandle();

    for(int i = 0; i<6; i++)
    {
        goal[i] = 0;
    }
    //joySubscriber = n->subscribe("joy", 100, &KinematicChainsNode::joyCallback, this);

    br = new tf::TransformBroadcaster();
    manipulator = new Manipulator();

    service1 = n->advertiseService("calculateIK",&KinematicChainsNode::ikService, this);
    service2 = n->advertiseService("changeEffectorDescription",&KinematicChainsNode::changeEffectorService, this);
    service3 = n->advertiseService("initJoints",&KinematicChainsNode::jointInitializationService, this);

    // parsing joint names
    const char* arm_base = "link1";
    const char* arm_tip = "link5";
    const char* tool_base = "link1";
    const char* tool_tip = "link5";

    for(int i = 1; i< argc; i++)
    {
        if( strcmp(argv[i],"-arm_base") == 0){arm_base = argv[i+1];}
        else if( strcmp(argv[i],"-arm_tip") == 0){arm_tip = argv[i+1];}
        else if( strcmp(argv[i],"-tool_base") == 0){tool_base = argv[i+1];}
        else if( strcmp(argv[i],"-tool_tip") == 0){tool_tip = argv[i+1];}
    }

    for(int i = 1; i< argc; i++)
    {
        if( strcmp(argv[i],"-arm") == 0){
            std::cout<<"initialaizng arm with param: "<<argv[i+1]<<", joints: "<<arm_base<<", and "<<arm_tip<<std::endl;
            parseRobotDescription(argv[i+1], Arm, arm_base, arm_tip);
        }
        else if( strcmp(argv[i],"-tool") == 0){
            std::cout<<"initialaizng effector with param: "<<argv[i+1]<<", joints: "<<tool_base<<", and "<<tool_tip<<std::endl;
            parseRobotDescription(argv[i+1], Effector, tool_base, tool_tip);
        }
    }

    if(!manipulator->isArmInitialized())
    {
        ROS_ERROR("Arm of the manipulator not initialized!");
        ROS_ERROR("\033[1;92mUse \"-arm\" option to provide a parameter name containing a valid URDF arm description.\033[0m");
        ROS_ERROR("Canceling.");
        exit(-2);
    }
    ROS_INFO("Node inited");
}

bool KinematicChainsNode::jointInitializationService(kinematic_chains::InitJoints::Request &req, kinematic_chains::InitJoints::Response &res)
{
    bool _arm = false, _effector = false;
    if(req.manipulator_part.compare("both") == 0)
    {
        _arm = _effector = true;
        ROS_INFO("initializing joints of the arm and the tool");
        if(req.jointCoordinates.capacity() != manipulator->getJnts().rows())
        {
            ROS_INFO("Expected number of joint coordinates: %d, number of coordintes provided: %d", manipulator->getJnts().rows(), (int)req.jointCoordinates.capacity() );
            ROS_ERROR("Initialization of manipulator joints failed!");
            return false;
        }
    }
    else if(req.manipulator_part.compare("arm") == 0)
    {
        _arm = true;
        ROS_INFO("initializing joints of the arm");
        if(req.jointCoordinates.capacity() != manipulator->getNrOfArmJoints())
        {
            ROS_INFO("Expected number of joint coordinates: %d, number of coordintes provided: %d", manipulator->getNrOfArmJoints(), (int)req.jointCoordinates.capacity() );
            ROS_ERROR("Initialization of arm joints failed!");
            return false;
        }
    }
    else if(req.manipulator_part.compare("effector") == 0)
    {
        _effector = true;
        ROS_INFO("initializing joints of the tool");
        if(req.jointCoordinates.capacity() != manipulator->getNrOfEffectorJoints())
        {
            ROS_INFO("Expected number of joint coordinates: %d, number of coordintes provided: %d", manipulator->getNrOfEffectorJoints(), (int)req.jointCoordinates.capacity() );
            ROS_ERROR("Initialization of effector joints failed!");
            return false;
        }
    }
    else
    {
        ROS_ERROR("No manipulator part defined! Define a part you want to initialize (arm, effector, both). Initialization of effector joints failed!");
        return false;
    }

    if(_effector && !manipulator->isEffectorInitialized())
    {
        ROS_ERROR("Effector not initialized. Initialization of effector joints failed!");
        return false;
    }
    if(_arm && !manipulator->isArmInitialized())
    {
        ROS_ERROR("Arm not initialized. Initialization of arm joints failed!");
        return false;
    }

    JntArray joints = manipulator->getJnts();

    ROS_INFO("overall joints number: %d", joints.rows());

    //current_q = JntArray(chain.getNrOfJoints());
    if(_arm)
        for(int i = 0; i<manipulator->getNrOfArmJoints();i++)
        {
            ROS_INFO("setting joint %d to %f", i,  req.jointCoordinates[i]);
            joints(i) = req.jointCoordinates[i];
        }

    if(_effector)
        for(int i = 0; i<manipulator->getNrOfEffectorJoints();i++)
        {
            ROS_INFO("setting effector joint %d, manipulator jonit %d to %f", i, i + manipulator->getNrOfArmJoints(), req.jointCoordinates[i+(_arm?manipulator->getNrOfArmJoints():0)]);
            joints(i+manipulator->getNrOfArmJoints()) = req.jointCoordinates[i+(_arm?manipulator->getNrOfArmJoints():0)];
        }

    manipulator->setJoints(joints);

    ROS_INFO("Joint coordinates initialized");
    res.return_value = true;
}

bool KinematicChainsNode::changeEffectorService(kinematic_chains::ChangeEffectorDescription::Request &req, kinematic_chains::ChangeEffectorDescription::Response &res)
{

    res.return_value = parseRobotDescription(req.parameter_name.c_str(), Effector, req.base_joint.c_str(), req.tip_joint.c_str());
}

bool KinematicChainsNode::ikService(kinematic_chains::CalculateIK::Request &req, kinematic_chains::CalculateIK::Response &res)
{
    if(!manipulator->isInitialized())
    {
        ROS_ERROR("Manipulator has to be initialized before IK request");
        return false;
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(req.goal.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //std::cout<<roll<<";\t"<<pitch<<";\t"<<yaw<<std::endl;
    double x, y, z;
    x = req.goal.position.x;
    y = req.goal.position.y;
    z = req.goal.position.z;

    //KDL::Frame destination;
    destination.p[0] = x;
    destination.p[1] = y;
    destination.p[2] = z;

    destination.M = KDL::Rotation::RPY(roll, pitch, yaw);
    //destination.M.RPY()

    ROS_INFO("ik requested, goal coordinates:\nx:%.2f,  \ty:%.2f,  \tz:%.2f,\nroll:%.2f,\tpitch:%.2f,\tyaw:%.2f,\t", x,y,z, roll, pitch, yaw);
    if(manipulator->calculateIK(destination))
    {
        KDL::JntArray jnts = manipulator->getJnts();
        sensor_msgs::JointState joint_state;

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(jnts.rows());
        joint_state.position.resize(jnts.rows());
        for(int i = 0; i< jnts.rows(); i++)
        {
            joint_state.position[i] = jnts(i);
            joint_state.name[i] = manipulator->getJntName(i).c_str();
        }

        res.jntCoordinates = joint_state;

        return true;
    }
    else return false;
}

void KinematicChainsNode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    goal[0] = 0.5*msg->axes[0];
    goal[1] = 0.5*msg->axes[1];
    goal[2] = -0.5*(msg->axes[2] - 2);
    goal[3] = 0.5*msg->axes[3];
    goal[4] = 0.5*msg->axes[4];
    goal[5] = -0.25*(msg->axes[5]-1);
}

void KinematicChainsNode::addNewManipulator(KDL::Chain arm, KDL::Chain effector)
{
    manipulator = new Manipulator(arm, effector);
}

void KinematicChainsNode::spin()
{
    ros::Rate loop_rate(20);
    while (ros::ok())
    {

        //to be removed

        //////////////////////////////////////////////////////
//        destination.p[0] = goal[0];
//        destination.p[1] = goal[1];
//        destination.p[2] = goal[2];

//        double x,y,z,w;
//        destination.M.Quaternion(x,y,z,w);
//        destination.M.DoRotY(goal[3]);
//        destination.M.DoRotX(goal[4]);
//        destination.M.DoRotZ(goal[5]);

//        publishGoal();

//        manipulator->calculateIK(destination);
        publishForwardKinematics();

        //////////////////////////////////////////////////////

        ros::spinOnce();

        loop_rate.sleep();
    }
}

void KinematicChainsNode::publishGoal()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(destination.p[0], destination.p[1], destination.p[2]));
    double x,y,z,w;
    destination.M.GetQuaternion(x,y,z,w);
    tf::Quaternion quat;
    quat.setValue(x,y,z,w);
    transform.setRotation(quat);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "goal"));
}

void KinematicChainsNode::publishForwardKinematics()
{
    KDL::Frame cartpos;
    KDL::JntArray jnt = manipulator->getJnts();

    KDL::Chain chain = manipulator->getChain();
    int j = 0;
    for(int k = 0; k < chain.getNrOfSegments(); k++)
    {
        if( chain.getSegment(k).getJoint().getType()!=KDL::Joint::None){
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
        s2 <<chain.getSegment(k).getJoint().getName()<<"_"<<k;
        if(k == 0)
            s1<< "base_link";
        else
            s1 << chain.getSegment(k-1).getJoint().getName()<<"_"<<k-1;
        
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), s1.str().c_str(), s2.str().c_str()));
    }
}

bool KinematicChainsNode::chainFromFile(std::string filename, KDL::Chain& outChain)
{
    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(filename, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    // to be finihed!
    return true;
}

bool KinematicChainsNode::parseRobotDescription(std::string parameterName, manipulator_part part, const char* root_name, const char* tip_name)
{
    KDL::Tree tree;
    std::string robot_desc_string;
    n->param(parameterName, robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)){
       ROS_ERROR("Failed to construct kdl tree from parameter '%s'", parameterName.c_str());
       return false;
    }
    Chain chain;
    if(! tree.getChain(root_name, tip_name,chain)){
        ROS_ERROR("Failed to extract chain with root %s and tip %s form tree", root_name, tip_name);
        return false;
    }
    if(part == Arm)
    {
        manipulator->setArm(chain);
    }
    else if(part == Effector)
    {
        manipulator->setEffector(chain);
    }
    return true;
}


