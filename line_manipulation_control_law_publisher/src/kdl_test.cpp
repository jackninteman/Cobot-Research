#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    // Init this ros node
    ros::init(argc, argv, "kdl_tester");
    
    // Setup ros node handle as unique pointer pointed to heap section
    // This node handle will be there as long as the main function keeps spinning
    std::unique_ptr<ros::NodeHandle> ros_node(new ros::NodeHandle);

    // Set up tree
    KDL::Tree 	kdl_tree_;
	KDL::Chain	kdl_chain_;
    std::unique_ptr<KDL::ChainDynParam> id_solver_;
    KDL::JntArray G_;
    KDL::JntArray q_;
	KDL::Vector gravity_;

    std::string robot_desc_string;
    ros_node->param("robot_description", robot_desc_string, std::string());

    
    // kdl parser
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    
    // kdl chain
    std::string root_name = "world";
	std::string tip_name = "end_effector_link";
	if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR("Failed to construct kdl chain");
        return false;
    }
    
    // This is a gravity 3D vector. Value is [0,0,-9.81]^T 
    gravity_ = KDL::Vector::Zero();
	gravity_(2) = -9.81;
    std::cout << "gravity_[2]: " << gravity_[2] << std::endl;
	G_.resize(4);
    q_.resize(4);
    q_.data << 0,0,0,0;


    // inverse dynamics solver
	id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

    // compute gravity torque
	std::cout << id_solver_->JntToGravity(q_, G_) << std::endl;
    std::cout << "G_: " << G_.data << std::endl;
    
    q_.data << 0,0.785,0,0;
    std::cout << id_solver_->JntToGravity(q_, G_) << std::endl;
    std::cout << "G_: " << G_.data << std::endl;

    q_.data << 0,-0.785,0,0;
    std::cout << id_solver_->JntToGravity(q_, G_) << std::endl;
    std::cout << "G_: " << G_.data << std::endl;
}