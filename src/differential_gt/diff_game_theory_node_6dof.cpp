#include "ros/ros.h"
#include <differential_gt/cgt.h>
#include <differential_gt/ncgt.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> 
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>

double alpha = 0.1;

/*
 The geometry_msgs/TwistStamped message type has inside a geometry_msgs/Twist message type, 
 which expresses velocity in free space broken into its linear and angular parts.
 The geometry_msgs/WrenchStamped message type has inside a geometry_msgs/Wrench message type, which 
 represents force in free space, separated into its linear and angular parts
*/

std::vector<double> range(double min, double max, double dt)
{
     /*From the public member function std::vector::push_back.

          void push_back (const value_type& val);

     This function add a new element at the end of the vector, after its current last element.
     The content of val is copied (or moved) to the new element.*/

     std::vector<double> range;
     for(int i=0; i<max/dt; i++) 
     {
          range.push_back(min + i*dt);
     }
     return range;
}

// This function is used to convert the three rotation dofs (roll, pitch, yaw) into their corresponding quaternion form, based on the rotation sequence XYZ.
Eigen::VectorXd from_euler_to_quaternion(double roll, double pitch, double yaw)
{
     // Definition of a rotation matrix based on the values of the three rotations
     Eigen::Matrix3d m;

     m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

     // Definition of the quaternion matrix based on the previous matrix definition
     Eigen::Quaterniond rot_mat_in_quaternion(m);

     // // Print the previous matrix in different ways
     // std::cout << "rot_mat_in_quaternion matrix form: \n" << rot_mat_in_quaternion.matrix() << "\n";
     // std::cout << "rot_mat_in_quaternion.vec() form: \n" << rot_mat_in_quaternion.vec() << "\n";
     // std::cout << "rot_mat_in_quaternion.w() form: \n" << rot_mat_in_quaternion.w() << "\n";

     // Defining the vector that stores the quaternion values
     Eigen::VectorXd quaternion_values; quaternion_values.resize(4);
     quaternion_values << rot_mat_in_quaternion.vec()[0],
                          rot_mat_in_quaternion.vec()[1],
                          rot_mat_in_quaternion.vec()[2],
                          rot_mat_in_quaternion.w();

     return quaternion_values;
}

// Callback function used for receiving the alpha parameter externally.
void alphaCallback(const std_msgs::Float32::ConstPtr& msg)
{
     ROS_INFO("I heard: [%f]", msg->data);
     alpha = msg->data;
}

int main(int argc, char **argv)
{    
     // Defining the ros node. The third argument is the name of the node
     ros::init(argc, argv, "diff_game_theory_node");
     
     /*NodeHandle is the main access point to communications with the ROS system.
     The first Nodehandle constructed will fully initialize this node;
     the last NodeHandle destructed will close down the node.*/
     ros::NodeHandle n;

     // This command gives the opportunity to read the publication of the alpha parameter in a topic as a subscriber.
     ros::AsyncSpinner spinner(4);
     spinner.start();
     
     // In this case, the n_dofs variable is extended to 6. The parameters that are modified come from the 
     // gt_traj_arbitration package of Paolo Franceschi's repository (https://github.com/paolofrance/gt_traj_arbitration)
     int n_dofs = 6;
     double dt = 0.01;
     
     // In this case, M_PI = 3.14159 
     std::vector<double> time = range(0.0, 6 * M_PI - dt, dt);

     // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
     Eigen::VectorXd ref_h; ref_h.resize(time.size());
     Eigen::VectorXd ref_r; ref_r.resize(time.size());
     
     // We define a constant reference to be followed by the robot
     /* 
      We will have three intervals of 2*pi here
      (1) In the first interval, the robot reference will be zero.
      (2) In the second interval, the robot reference will be 0.5
      (3) In the last interval, the robot reference will be the same of the human one.
     */

     for (int i = 0; i<time.size(); i++)
     {
          ref_h(i) = 1;
          ref_r(i) = 0;
     }

     // Defining the Identity and Null Matrices
     Eigen::MatrixXd O; O.resize(n_dofs, n_dofs); O.setZero();
     Eigen::MatrixXd I; I.resize(n_dofs, n_dofs); I.setIdentity();

     // Defining the REF_H matrix that stores, in each column, the human reference defined above for each dof (6).
     Eigen::MatrixXd REF_H; REF_H.resize(ref_h.size(), n_dofs);
     REF_H << ref_h, ref_h, ref_h, ref_h, ref_h, ref_h;
     // ROS_INFO_STREAM("REF_H: \n" << REF_H << "\n");

     // Defining the REF_R matrix that stores, in each column, the robot reference defined above for each dof (6).
     Eigen::MatrixXd REF_R; REF_R.resize(ref_r.size(), n_dofs);
     REF_R << ref_r, ref_r, ref_r, ref_r, ref_r, ref_r;
     // ROS_INFO_STREAM("REF_R: \n" << REF_R << "\n");

     // Here the system matrices are defined
     Eigen::MatrixXd Ac; Ac.resize(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Bc; Bc.resize(2*n_dofs,n_dofs);
     Eigen::MatrixXd Cc; Cc.resize(n_dofs,2*n_dofs);

     // The mass, damping and stiffness matrices are defined
     Eigen::MatrixXd M; M.resize(n_dofs, n_dofs);
     Eigen::MatrixXd D; D.resize(n_dofs, n_dofs);
     Eigen::MatrixXd K; K.resize(n_dofs, n_dofs);

     // Inizialization of system matrices
     M << 10*I;

     K << O;
     
     D << 25, 0, 0, 0, 0, 0,
          0, 25, 0, 0, 0, 0,
          0, 0, 25, 0, 0, 0,
          0, 0, 0, 10, 0, 0,
          0, 0, 0, 0, 10, 0,
          0, 0, 0, 0, 0, 10; 

     // Initialize the linearized state space matrices
     Ac << O, I,
           -M.inverse()*K, -M.inverse()*D;

     Bc << O,
           M.inverse();

     /* SYSTEM PARAMETERS*/

     // Initialize the Cooperative GT controller, set the System Parameters and get them.
     CoopGT cgt(n_dofs,dt);
     cgt.setSysParams(Ac,Bc);
     cgt.getSysParams(Ac,Bc,Cc);

     // Initialize the Non-cooperative GT controller, set the System Parameters and get them.
     NonCoopGT ncgt(n_dofs,dt);
     ncgt.setSysParams(Ac,Bc);
     ncgt.getSysParams(Ac,Bc,Cc);
 
     /* COST PARAMETERS */
     
     // Initialize the Cooperative GT controller state-error-weight cost matrices. They are also used in the Non-cooperative GT controller as it is written in the Franceschi's paper.
     Eigen::MatrixXd Qhh; Qhh.resize(2*n_dofs,2*n_dofs); 
     Eigen::MatrixXd Qhr; Qhr.resize(2*n_dofs,2*n_dofs); 
     Eigen::MatrixXd Qrr; Qrr.resize(2*n_dofs,2*n_dofs); 
     Eigen::MatrixXd Qrh; Qrh.resize(2*n_dofs,2*n_dofs); 

     // Initialize the Cooperative GT controller weighted state-error-weight cost matrices. They are also used in the Non-cooperative GT controller as it is written in the Franceschi's paper.
     Eigen::MatrixXd Qh; Qh.resize(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Qr; Qr.resize(2*n_dofs,2*n_dofs);
     
     /* FILL THE STATE-ERROR-WEIGHT COST MATRICES*/
  
     // Human state-error-weight cost component based on human references. 
     // I also checked another paper of Paolo Franceschi at this link: https://arxiv.org/pdf/2307.10739
     Qhh << I, O,
            O, 0.0001*I;

     // Human state-error-weight cost component based on robot references.
     // I also checked another paper of Paolo Franceschi at this link: https://arxiv.org/pdf/2307.10739
     Qhr << O, O,
            O, O;

     // Robot state-error-weight cost component based on robot references.
     Qrr << I, O,
            O, 0.0001*I;

     // Robot state-error-weight cost component based on human references.
     Qrh << O, O,
            O, O;
     
     // Initialize the control-input cost matrices
     Eigen::MatrixXd Rh; Rh.resize(n_dofs,n_dofs); Rh << 0.0005*I; 
     Eigen::MatrixXd Rr; Rr.resize(n_dofs,n_dofs); Rr << 0.0005*I;

     // Subscribing to a topic called '/alpha' so that we can have the alpha parameter coming from an external node. 
     ros::Subscriber alpha_sub = n.subscribe("/alpha", 1000, alphaCallback);
     cgt.setAlpha(alpha);

     /* SET THE DIFF GAME THEORY PARAMETERS*/

     // Set the Cooperative cost parameters
     cgt.setCostsParams(Qhh,Qhr,Qrh,Qrr,Rh,Rr);

     // Get Cost Matrices Qh, Qr, Rh, Rr. These are the matrices that will be passed in the Non-cooperative case for the reasoning commented above.
     cgt.getCostMatrices(Qh,Qr,Rh,Rr);

     // Set the Non-cooperative cost parameters, based on the ones got from the previous line.
     ncgt.setCostsParams(Qh,Qr,Rh,Rr);

     // Get the Non-cooperative matrices.
     ncgt.getCostMatrices(Qh,Qr,Rh,Rr);

     /* CURRENT STATE*/

     // Initialize the Current State
     Eigen::VectorXd Z = Eigen::VectorXd::Zero(2*n_dofs);
     Eigen::VectorXd dZ = Eigen::VectorXd::Zero(2*n_dofs);

     // Set the Cooperative current State
     cgt.setCurrentState(Z);

     // Set the Non-cooperative current state
     ncgt.setCurrentState(Z);

     // Get the Cooperative current state
     Z = cgt.getCurrentState();

     // Get the Non-cooperative current state
     Z = ncgt.getCurrentState();
  
     // The following method computes the Cooperative gain Kgt
     cgt.computeCooperativeGains();

     // The following method computes the Non-Cooperative gains Kh and Kr
     ncgt.computeNonCooperativeGains();

     // Initialize and the Cooperative gain   
     Eigen::MatrixXd Kgt = cgt.getCooperativeGains();

     // Get the Non-Cooperative gains
     Eigen::MatrixXd Kh,Kr;
     ncgt.getNonCooperativeGains(Kh,Kr); 
  
     /* IN HERE, WE DEFINE A FIRST POSITIONAL REFERENCE TO OUR CONTROLLER */ 

     // In Eigen::VectorXd, the .segment() block operation can specify a
     // a block containing n elements, starting at position i following
     // this structure: vector.segment(i,n);

     // Since now the references for all the dofs are defined in a matrix, 
     // to get the value of the first row (corresponding to the first reference value for each dof),
     // the .block() function is used.
     // For dynamic-size block expression, the definition is matrix.block(i,j,p,q)
     // Corresponding to a block of size (p,q), starting at (i,j). i and j indexes, as always in Eigen, start at 0.
     // Since Eigen::VectorXd saves the element as a column vector and the firsts references for each dof are in correspondence of the first row, the .transpose() function is used.

     Eigen::VectorXd rh = REF_H.block(0,0,1,n_dofs).transpose();
     Eigen::VectorXd rr = REF_R.block(0,0,1,n_dofs).transpose();

     // I realized that if I want to print Eigen::VectorXd, to see the whole vector it is better to use std::cout because using ROS_INFO_STREAM, it shows just the first element.
     // std::cout << "Eigen::VectorXd rh: \n" << rh << "\n";
     // std::cout << "Eigen::VectorXd rr: \n" << rr << "\n";

     // setPosReference for the Cooperative case
     cgt.setPosReference(rh,rr);  

     // setPosReference for the Non-cooperative case
     ncgt.setPosReference(rh,rr);

     // Get the first weighted reference for the Cooperative case
     Eigen::VectorXd weighted_reference;
     weighted_reference = cgt.getReference();

     // Get firsts human and robot reference for the Non-cooperative case
     ncgt.getReference(rh,rr);

     // Instantiate ROS state messages

     geometry_msgs::PoseStamped state_pose_msg;
     geometry_msgs::TwistStamped state_velocity_msg;

     geometry_msgs::PoseStamped human_reference_pose_msg;
     geometry_msgs::TwistStamped human_reference_velocity_msg;

     geometry_msgs::PoseStamped robot_reference_pose_msg;
     geometry_msgs::TwistStamped robot_reference_velocity_msg;

     geometry_msgs::PoseStamped weighted_reference_pose_msg;
     geometry_msgs::TwistStamped weighted_reference_velocity_msg;

     // Instantiate ROS control messages

     geometry_msgs::WrenchStamped optimal_control_robot_msg;
     geometry_msgs::WrenchStamped optimal_control_human_msg;

     geometry_msgs::WrenchStamped optimal_control_human_weighted_msg;
     geometry_msgs::WrenchStamped optimal_control_robot_weighted_msg;

     // Instantiate ROS publishers

     // Instead of considering the /state/pose topic, we will consider the topic which the impedance controller is subscribed to

     ros::Publisher state_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 1);
     ros::Publisher state_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/state/velocity", 1);

     ros::Publisher human_reference_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/reference/human/pose", 1);
     ros::Publisher human_reference_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/reference/human/velocity", 1);

     ros::Publisher robot_reference_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/reference/robot/pose", 1);
     ros::Publisher robot_reference_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/reference/robot/velocity", 1);

     ros::Publisher weighted_reference_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/reference/weighted/pose", 1);
     ros::Publisher weighted_reference_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/reference/weighted/velocity", 1);

     ros::Publisher optimal_control_robot_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/robot", 1);
     ros::Publisher optimal_control_human_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/human", 1);

     ros::Publisher optimal_control_human_weighted_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/human_weighted", 1);
     ros::Publisher optimal_control_robot_weighted_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/robot_weighted", 1);

     // Create a ROS loop rate
     ros::Rate control_rate(1/dt);

     // Create a ROS time reference from the starting moment
     ros::Time starting_time = ros::Time::now();
     ros::Time seconds_from_start;

     // Create a control object store future optimal control inputs for the Cooperative case
     Eigen::VectorXd coop_control;
     cgt.computeControlInputs();
     cgt.getControlInput(coop_control);

     // Create a control object store future optimal control inputs for the Non-cooperative case
     Eigen::VectorXd non_coop_control;
     ncgt.computeControlInputs();
     ncgt.getControlInput(non_coop_control);

     // Indexes initialization
     int reference_index = 0;
     long double current_time = 0;

     ROS_INFO_STREAM("The controller is initialized. The demo starts now.");

     // Main loop
     while (ros::ok())
     {
          // All these functions are placed here in order to re-compute the values of the gain matrices and the corresponding control inputs
          // Depeding on the value of the alpha parameter that is passed through a topic.
          cgt.setAlpha(alpha);
          cgt.setCostsParams(Qhh,Qhr,Qrh,Qrr,Rh,Rr);
          cgt.getCostMatrices(Qh,Qr,Rh,Rr);
          ncgt.setCostsParams(Qh,Qr,Rh,Rr);
          ncgt.getCostMatrices(Qh,Qr,Rh,Rr);
          cgt.computeCooperativeGains();
          ncgt.computeNonCooperativeGains();
          Kgt = cgt.getCooperativeGains();
          ncgt.getNonCooperativeGains(Kh,Kr);
          cgt.setPosReference(rh,rr); 
          ncgt.setPosReference(rh,rr);
          weighted_reference = cgt.getReference();
          ncgt.getReference(rh,rr);

          reference_index += 1;
          reference_index = (reference_index % time.size());
          // ROS_INFO_STREAM("Reference index is %d", reference_index);

          current_time += dt;

          rh = REF_H.block(reference_index, 0, 1, n_dofs).transpose();
          rr = REF_R.block(reference_index, 0, 1, n_dofs).transpose();

          // Set the current positional reference for the Cooperative case
          cgt.setPosReference(rh, rr);

          // Set the current positional reference for the Non-cooperative case
          ncgt.setPosReference(rh,rr);

          // We need to update the state with the real robot data. In this case, 
          // we will update with the last known state.

          // For the Cooperative case
          Eigen::VectorXd cgt_state = cgt.getCurrentState();
          cgt.setCurrentState(cgt_state);

          // For the Non-cooperative case
          Eigen::VectorXd ncgt_state = ncgt.getCurrentState();
          ncgt.setCurrentState(ncgt_state);          

          // This step function assumes that the optimal control will be performed by
          // both human and robot. In the future, we will create a new step() method
          // that accepts a control input from one of the agents. We assume that:
          //   1 : reference/control human
          //   2 : reference/control robot
          //
          cgt.step(cgt_state, rh, rr);
          ncgt.step(ncgt_state, rh, rr);

          // We retrieve the optimal control inputs from before the state has been
          // performed. This command is performed also inside the ncgt.step(). The 
          // optimal control inputs are calculated based on the current state.
          cgt.getControlInput(coop_control);
          ncgt.getControlInput(non_coop_control);

          ROS_INFO_STREAM("Coop control input: " << coop_control.transpose());
          ROS_INFO_STREAM("Non-coop Control input: " << non_coop_control.transpose());

          weighted_reference = cgt.getReference();
          ncgt.getReference(rh,rr);

          ROS_INFO_STREAM("weighted_reference: " << weighted_reference.transpose());
          ROS_INFO_STREAM("human reference: " << rh.transpose());
          ROS_INFO_STREAM("robot reference: " << rr.transpose());

          // Note that we print the state stored before the step has been done.
          // In other words, we print the previous state. 
          ROS_INFO_STREAM("cgt_state: " << cgt_state.transpose());
          ROS_INFO_STREAM("ncgt_state: " << ncgt_state.transpose());

          // Update time
          seconds_from_start = ros::Time(current_time); // time[reference_index];

          // Update time from message headers
          state_pose_msg.header.stamp = seconds_from_start;
          state_velocity_msg.header.stamp = seconds_from_start;

          human_reference_pose_msg.header.stamp = seconds_from_start;
          human_reference_velocity_msg.header.stamp = seconds_from_start;

          robot_reference_pose_msg.header.stamp = seconds_from_start;
          robot_reference_velocity_msg.header.stamp = seconds_from_start;

          weighted_reference_pose_msg.header.stamp = seconds_from_start;
          weighted_reference_velocity_msg.header.stamp = seconds_from_start;

          optimal_control_human_msg.header.stamp = seconds_from_start;
          optimal_control_robot_msg.header.stamp = seconds_from_start;

          optimal_control_human_weighted_msg.header.stamp = seconds_from_start;
          optimal_control_robot_weighted_msg.header.stamp = seconds_from_start;

          // Update the reference messages. They are positional references only.
          
          // Human reference positions
          human_reference_pose_msg.pose.position.x = rh(0);
          human_reference_pose_msg.pose.position.y = rh(1);
          human_reference_pose_msg.pose.position.z = rh(2);

          // Human reference orientations
          Eigen::Vector4d human_reference_orientation_quaternion;
          human_reference_orientation_quaternion = from_euler_to_quaternion(rh(3), rh(4), rh(5));
          human_reference_pose_msg.pose.orientation.x = human_reference_orientation_quaternion[0];
          human_reference_pose_msg.pose.orientation.y = human_reference_orientation_quaternion[1];
          human_reference_pose_msg.pose.orientation.z = human_reference_orientation_quaternion[2];
          human_reference_pose_msg.pose.orientation.w = human_reference_orientation_quaternion[3];


          // Robot reference positions
          robot_reference_pose_msg.pose.position.x = rr(0);
          robot_reference_pose_msg.pose.position.y = rr(1);
          robot_reference_pose_msg.pose.position.z = rr(2);

          // Robot reference orientations
          Eigen::Vector4d robot_reference_orientation_quaternion;
          robot_reference_orientation_quaternion = from_euler_to_quaternion(rr(3), rr(4), rr(5));
          robot_reference_pose_msg.pose.orientation.x = robot_reference_orientation_quaternion[0];
          robot_reference_pose_msg.pose.orientation.y = robot_reference_orientation_quaternion[1];
          robot_reference_pose_msg.pose.orientation.z = robot_reference_orientation_quaternion[2];
          robot_reference_pose_msg.pose.orientation.w = robot_reference_orientation_quaternion[3];

          // Weighted reference positions
          weighted_reference_pose_msg.pose.position.x = weighted_reference(0);
          weighted_reference_pose_msg.pose.position.y = weighted_reference(1);
          weighted_reference_pose_msg.pose.position.z = weighted_reference(2);

          // Weighted reference orientations
          Eigen::Vector4d weighted_reference_orientation_quaternion;
          weighted_reference_orientation_quaternion = from_euler_to_quaternion(weighted_reference(3), weighted_reference(4), weighted_reference(5));
          weighted_reference_pose_msg.pose.orientation.x = weighted_reference_orientation_quaternion[0];
          weighted_reference_pose_msg.pose.orientation.y = weighted_reference_orientation_quaternion[1];
          weighted_reference_pose_msg.pose.orientation.z = weighted_reference_orientation_quaternion[2];
          weighted_reference_pose_msg.pose.orientation.w = weighted_reference_orientation_quaternion[3];


          // Update state pose message.
          if (alpha >= 0.5)
          {
               state_pose_msg.pose.position.x = cgt_state(0);
               state_pose_msg.pose.position.y = cgt_state(1);
               state_pose_msg.pose.position.z = cgt_state(2);
               Eigen::Vector4d state_pose_orientation_quaternion;
               state_pose_orientation_quaternion = from_euler_to_quaternion(cgt_state(3), cgt_state(4), cgt_state(5));
               state_pose_msg.pose.orientation.x = state_pose_orientation_quaternion[0];
               state_pose_msg.pose.orientation.y = state_pose_orientation_quaternion[1];
               state_pose_msg.pose.orientation.z = state_pose_orientation_quaternion[2];
               state_pose_msg.pose.orientation.w = state_pose_orientation_quaternion[3];
          }

          else if (alpha < 0.5)
          {
               state_pose_msg.pose.position.x = ncgt_state(0);
               state_pose_msg.pose.position.y = ncgt_state(1);
               state_pose_msg.pose.position.z = ncgt_state(2);
               Eigen::Vector4d state_pose_orientation_quaternion;
               state_pose_orientation_quaternion = from_euler_to_quaternion(ncgt_state(3), ncgt_state(4), ncgt_state(5));
               state_pose_msg.pose.orientation.x = state_pose_orientation_quaternion[0];
               state_pose_msg.pose.orientation.y = state_pose_orientation_quaternion[1];
               state_pose_msg.pose.orientation.z = state_pose_orientation_quaternion[2];
               state_pose_msg.pose.orientation.w = state_pose_orientation_quaternion[3];
          }

          // Update state velocity message.
          // Since it is a unidimensional problem, we will use only the Y positional axis. All the rest, leave as zero.
          if (alpha >= 0.5)
          {
               state_velocity_msg.twist.linear.x = cgt_state(6);
               state_velocity_msg.twist.linear.y = cgt_state(7);
               state_velocity_msg.twist.linear.y = cgt_state(8);
               state_velocity_msg.twist.angular.x = cgt_state(9);
               state_velocity_msg.twist.angular.y = cgt_state(10);
               state_velocity_msg.twist.angular.z = cgt_state(11);
          }

          else if (alpha < 0.5)
          {
               state_velocity_msg.twist.linear.x = ncgt_state(6);
               state_velocity_msg.twist.linear.y = ncgt_state(7);
               state_velocity_msg.twist.linear.y = ncgt_state(8);
               state_velocity_msg.twist.angular.x = ncgt_state(9);
               state_velocity_msg.twist.angular.y = ncgt_state(10);
               state_velocity_msg.twist.angular.z = ncgt_state(11);
          }

          // Update the control messages.
          optimal_control_human_msg.wrench.force.x = non_coop_control(0);
          optimal_control_human_msg.wrench.force.y = non_coop_control(1);
          optimal_control_human_msg.wrench.force.z = non_coop_control(2);
          optimal_control_human_msg.wrench.torque.x = non_coop_control(3);
          optimal_control_human_msg.wrench.torque.y = non_coop_control(4);
          optimal_control_human_msg.wrench.torque.z = non_coop_control(5);

          optimal_control_robot_msg.wrench.force.x = non_coop_control(6);
          optimal_control_robot_msg.wrench.force.y = non_coop_control(7);
          optimal_control_robot_msg.wrench.force.z = non_coop_control(8);
          optimal_control_robot_msg.wrench.torque.x = non_coop_control(9);
          optimal_control_robot_msg.wrench.torque.y = non_coop_control(10);
          optimal_control_robot_msg.wrench.torque.z = non_coop_control(11);

          optimal_control_human_weighted_msg.wrench.force.x = coop_control(0);
          optimal_control_human_weighted_msg.wrench.force.y = coop_control(1);
          optimal_control_human_weighted_msg.wrench.force.z = coop_control(2);
          optimal_control_human_weighted_msg.wrench.torque.x = coop_control(3);
          optimal_control_human_weighted_msg.wrench.torque.y = coop_control(4);
          optimal_control_human_weighted_msg.wrench.torque.z = coop_control(5);

          optimal_control_robot_weighted_msg.wrench.force.x = coop_control(6);
          optimal_control_robot_weighted_msg.wrench.force.y = coop_control(7);
          optimal_control_robot_weighted_msg.wrench.force.z = coop_control(8);
          optimal_control_robot_weighted_msg.wrench.torque.x = coop_control(9);
          optimal_control_robot_weighted_msg.wrench.torque.y = coop_control(10);
          optimal_control_robot_weighted_msg.wrench.torque.z = coop_control(11);

          // Publish messages
          state_pose_pub.publish(state_pose_msg);
          state_velocity_pub.publish(state_velocity_msg);

          human_reference_pose_pub.publish(human_reference_pose_msg);
          human_reference_velocity_pub.publish(human_reference_velocity_msg);

          robot_reference_pose_pub.publish(robot_reference_pose_msg);
          robot_reference_velocity_pub.publish(robot_reference_velocity_msg);

          weighted_reference_pose_pub.publish(weighted_reference_pose_msg);
          weighted_reference_velocity_pub.publish(weighted_reference_velocity_msg);

          optimal_control_human_pub.publish(optimal_control_human_msg);
          optimal_control_robot_pub.publish(optimal_control_robot_msg);

          optimal_control_human_weighted_pub.publish(optimal_control_human_weighted_msg);
          optimal_control_robot_weighted_pub.publish(optimal_control_robot_weighted_msg);

          // Synchronize
          control_rate.sleep();
     }
     spinner.stop();
  return 0;
}





