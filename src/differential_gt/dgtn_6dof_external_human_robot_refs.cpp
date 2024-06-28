#include "ros/ros.h"
#include <differential_gt/cgt.h>
#include <differential_gt/ncgt.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> 
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <franka_msgs/FrankaState.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// First definition of the alpha value so that a first computation can be done.
double alpha = 0.1;

// Definition of the human and robot references that will be assigned through the subscription.
geometry_msgs::PoseStamped ref_h;
geometry_msgs::PoseStamped ref_r;

// n_dofs definition.
int n_dofs = 6;

// Initialize the Current State
Eigen::VectorXd Z = Eigen::VectorXd::Zero(2*n_dofs);
Eigen::VectorXd dZ = Eigen::VectorXd::Zero(2*n_dofs);

// Indicates if the first initial robot pose is received.
bool initial_robot_state_ok = false;


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

// This function is used to convert the three rotation dofs (roll, pitch, yaw) into their corresponding quaternion form, based on the rotation sequence RPY.
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

// This function is used to convert the quaternion values into their corresponding euler angles form, based on the rotation sequence RPY (roll, pitch, yaw).
Eigen::VectorXd from_quaternion_to_euler(double x, double y, double z, double w)
{
     // Defining a Eigen::Quaterniond in order to store the quaternion values
     Eigen::Quaterniond rot_mat_in_quaternion;
     rot_mat_in_quaternion.x() = x;
     rot_mat_in_quaternion.y() = y;
     rot_mat_in_quaternion.z() = z;
     rot_mat_in_quaternion.w() = w;

     // std::cout << "rot_mat_in_quaternion: \n" << rot_mat_in_quaternion.matrix() << "\n";

     // Defining the vector that stores the euler angles
     Eigen::VectorXd euler_angles; euler_angles.resize(3);
     euler_angles = rot_mat_in_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

     // std::cout << "euler_angles values: \n" << euler_angles << "\n";
     return euler_angles;
}

// Callback function used for receiving the alpha parameter externally.
void alphaCallback(const std_msgs::Float32::ConstPtr& msg)
{
     alpha = msg->data;
}

// Callback function used for receiving the human reference externally.
void human_refCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     ref_h.pose.position.x = msg->pose.position.x;
     ref_h.pose.position.y = msg->pose.position.y;
     ref_h.pose.position.z = msg->pose.position.z;
     // ref_h.pose.orientation.x = msg->pose.orientation.x;
     // ref_h.pose.orientation.y = msg->pose.orientation.y;
     // ref_h.pose.orientation.z = msg->pose.orientation.z;
     // ref_h.pose.orientation.w = msg->pose.orientation.w;
}

// Callback function used for receiving the robot reference externally.
void robot_refCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     ref_r.pose.position.x = msg->pose.position.x;
     ref_r.pose.position.y = msg->pose.position.y;
     ref_r.pose.position.z = msg->pose.position.z;
     // ref_r.pose.orientation.x = msg->pose.orientation.x;
     // ref_r.pose.orientation.y = msg->pose.orientation.y;
     // ref_r.pose.orientation.z = msg->pose.orientation.z;
     // ref_r.pose.orientation.w = msg->pose.orientation.w;
}


// Callback function used for receiving the initial state of the robot.
void current_robot_stateCallback(const franka_msgs::FrankaStateConstPtr& msg)
{
     if (initial_robot_state_ok)
          return;

     // A matrix with the values passed by msg is initialized, recreating the rotation matrix 
     Eigen::Matrix3d rot_mat;
     rot_mat << msg->O_T_EE[0], msg->O_T_EE[4], msg->O_T_EE[8],
                msg->O_T_EE[1], msg->O_T_EE[5], msg->O_T_EE[9],
                msg->O_T_EE[2], msg->O_T_EE[6], msg->O_T_EE[10];
     
     // Defining a Eigen::Quaterniond to use it as a bridge to convert the matrix to the corresponding euler angles.
     Eigen::Quaterniond rot_mat_quaternion(rot_mat);

     // Defining the vector that stores the euler angles 
     Eigen::VectorXd euler_angles; euler_angles.resize(3);
     euler_angles = rot_mat_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

     // Assignment of the relevant parameters to a geometry_msgs::PoseStamped type
     Z(0) = msg->O_T_EE[12];
     Z(1) = msg->O_T_EE[13];
     Z(2) = msg->O_T_EE[14];

     Z(3) = euler_angles(0);
     Z(4) = euler_angles(1);
     Z(5) = euler_angles(2);

     // Fill initial references with the current robot state
     ref_h.pose.position.x = Z(0);
     ref_h.pose.position.y = Z(1);
     ref_h.pose.position.z = Z(2);
     
     ref_r.pose.position.x = Z(0);
     ref_r.pose.position.y = Z(1);
     ref_r.pose.position.z = Z(2);
   
     ref_h.pose.orientation = tf2::toMsg(rot_mat_quaternion);
     ref_r.pose.orientation = tf2::toMsg(rot_mat_quaternion);

     initial_robot_state_ok = true;

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
     ros::AsyncSpinner spinner(5);
     spinner.start();

     // Subscribing to a topic called '/alpha' so that we can have the alpha parameter coming from an external node. 
     ros::Subscriber alpha_sub = n.subscribe("/alpha", 1, alphaCallback);
     // Subscribing to a topic called '/human_ref' so that we can have the human reference coming from an external node.
     ros::Subscriber human_ref_sub = n.subscribe("/human_ref", 1, human_refCallback);
     // Subscribing to a topic called '/robot_ref' so that we can have the robot reference coming from an external node.
     ros::Subscriber robot_ref_sub = n.subscribe("/robot_ref", 1, robot_refCallback);
     // Subscribing to a topic called '/franka_state_controller/franka_states' so that we can have the actual state of the robot.
     ros::Subscriber current_robot_state_sub = n.subscribe("/franka_state_controller/franka_states", 1, current_robot_stateCallback); 
     
     while(!initial_robot_state_ok)
     {
          ROS_INFO("waiting for an initial robot pose");
          ros::Duration(5).sleep();
     }

     // In this case, the n_dofs variable is extended to 6. The parameters that are modified come from the 
     // gt_traj_arbitration package of Paolo Franceschi's repository (https://github.com/paolofrance/gt_traj_arbitration)
     double dt = 0.01;
     
     // In this case, M_PI = 3.14159 
     // std::vector<double> time = range(0.0, 6 * M_PI - dt, dt);

     // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
     // Eigen::VectorXd ref_h; ref_h.resize(time.size());
     // Eigen::VectorXd ref_r; ref_r.resize(time.size());
     
     // We define a constant reference to be followed by the robot
     /* 
      We will have three intervals of 2*pi here
      (1) In the first interval, the robot reference will be zero.
      (2) In the second interval, the robot reference will be 0.5
      (3) In the last interval, the robot reference will be the same of the human one.
     */

     // for (int i = 0; i<time.size(); i++)
     // {
     //      ref_h(i) = 1;
     //      ref_r(i) = 0;
     // }

     // Defining the Identity and Null Matrices
     Eigen::MatrixXd O; O.resize(n_dofs, n_dofs); O.setZero();
     Eigen::MatrixXd I; I.resize(n_dofs, n_dofs); I.setIdentity();

     // Converting the quaternion values of human and robot references in euler angles.
     Eigen::VectorXd human_euler_angles; human_euler_angles.resize(3);
     human_euler_angles = from_quaternion_to_euler(ref_h.pose.orientation.x, ref_h.pose.orientation.y, ref_h.pose.orientation.z, ref_h.pose.orientation.w);
     Eigen::VectorXd robot_euler_angles; robot_euler_angles.resize(3);
     robot_euler_angles = from_quaternion_to_euler(ref_r.pose.orientation.x, ref_r.pose.orientation.y, ref_r.pose.orientation.z, ref_r.pose.orientation.w);
     
     std::cout << "human_euler_angles: \n" << human_euler_angles << "\n";
     std::cout << "robot_euler_angles: \n" << robot_euler_angles << "\n";

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

     // Initialize the Cooperative GT controlcurrent_robot_stateCallbackler weighted state-error-weight cost matrices. They are also used in the Non-cooperative GT controller as it is written in the Franceschi's paper.
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
     // I realized that if I want to print Eigen::VectorXd, to see the whole vector it is better to use std::cout because using ROS_INFO_STREAM, it shows just the first element.
     Eigen::VectorXd rh; rh.resize(n_dofs);
     rh << ref_h.pose.position.x, ref_h.pose.position.y, ref_h.pose.position.z, human_euler_angles[0], human_euler_angles[1], human_euler_angles[2];
     Eigen::VectorXd rr; rr.resize(n_dofs);
     rr << ref_r.pose.position.x, ref_r.pose.position.y, ref_r.pose.position.z, robot_euler_angles[0], robot_euler_angles[1], robot_euler_angles[2];

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

     geometry_msgs::PoseStamped commanded_pose_msg;
     geometry_msgs::TwistStamped commanded_velocity_msg;

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

     ros::Publisher commanded_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 1);
     ros::Publisher commanded_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/state/velocity", 1);

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
     std::cin.get();

     // Main loop
     while (ros::ok())
     {
          Eigen::VectorXd human_euler_angles; human_euler_angles.resize(3);
          human_euler_angles = from_quaternion_to_euler(ref_h.pose.orientation.x, ref_h.pose.orientation.y, ref_h.pose.orientation.z, ref_h.pose.orientation.w);
          Eigen::VectorXd robot_euler_angles; robot_euler_angles.resize(3);
          robot_euler_angles = from_quaternion_to_euler(ref_r.pose.orientation.x, ref_r.pose.orientation.y, ref_r.pose.orientation.z, ref_r.pose.orientation.w);
     
          // std::cout << "human_euler_angles: \n" << human_euler_angles << "\n";
          // std::cout << "robot_euler_angles: \n" << robot_euler_angles << "\n";

          rh << ref_h.pose.position.x, ref_h.pose.position.y, ref_h.pose.position.z, human_euler_angles[0], human_euler_angles[1], human_euler_angles[2];
          rr << ref_r.pose.position.x, ref_r.pose.position.y, ref_r.pose.position.z, robot_euler_angles[0], robot_euler_angles[1], robot_euler_angles[2];
          
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

          // reference_index += 1;
          // reference_index = (reference_index % time.size());
          // ROS_INFO_STREAM("Reference index is %d", reference_index);

          current_time += dt;

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
          // both human and robot.
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
          commanded_pose_msg.header.stamp = seconds_from_start;
          commanded_velocity_msg.header.stamp = seconds_from_start;

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
               commanded_pose_msg.pose.position.x = cgt_state(0);
               commanded_pose_msg.pose.position.y = cgt_state(1);
               commanded_pose_msg.pose.position.z = cgt_state(2);
               Eigen::Vector4d commanded_pose_orientation_quaternion;
               commanded_pose_orientation_quaternion = from_euler_to_quaternion(cgt_state(3), cgt_state(4), cgt_state(5));
               commanded_pose_msg.pose.orientation.x = commanded_pose_orientation_quaternion[0];
               commanded_pose_msg.pose.orientation.y = commanded_pose_orientation_quaternion[1];
               commanded_pose_msg.pose.orientation.z = commanded_pose_orientation_quaternion[2];
               commanded_pose_msg.pose.orientation.w = commanded_pose_orientation_quaternion[3];
          }

          else if (alpha < 0.5)
          {
               commanded_pose_msg.pose.position.x = ncgt_state(0);
               commanded_pose_msg.pose.position.y = ncgt_state(1);
               commanded_pose_msg.pose.position.z = ncgt_state(2);
               Eigen::Vector4d commanded_pose_orientation_quaternion;
               commanded_pose_orientation_quaternion = from_euler_to_quaternion(ncgt_state(3), ncgt_state(4), ncgt_state(5));
               commanded_pose_msg.pose.orientation.x = commanded_pose_orientation_quaternion[0];
               commanded_pose_msg.pose.orientation.y = commanded_pose_orientation_quaternion[1];
               commanded_pose_msg.pose.orientation.z = commanded_pose_orientation_quaternion[2];
               commanded_pose_msg.pose.orientation.w = commanded_pose_orientation_quaternion[3];
          }

          // Update state velocity message.
          // Since it is a unidimensional problem, we will use only the Y positional axis. All the rest, leave as zero.
          if (alpha >= 0.5)
          {
               commanded_velocity_msg.twist.linear.x = cgt_state(6);
               commanded_velocity_msg.twist.linear.y = cgt_state(7);
               commanded_velocity_msg.twist.linear.y = cgt_state(8);
               commanded_velocity_msg.twist.angular.x = cgt_state(9);
               commanded_velocity_msg.twist.angular.y = cgt_state(10);
               commanded_velocity_msg.twist.angular.z = cgt_state(11);
          }

          else if (alpha < 0.5)
          {
               commanded_velocity_msg.twist.linear.x = ncgt_state(6);
               commanded_velocity_msg.twist.linear.y = ncgt_state(7);
               commanded_velocity_msg.twist.linear.y = ncgt_state(8);
               commanded_velocity_msg.twist.angular.x = ncgt_state(9);
               commanded_velocity_msg.twist.angular.y = ncgt_state(10);
               commanded_velocity_msg.twist.angular.z = ncgt_state(11);
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
          commanded_pose_pub.publish(commanded_pose_msg);
          commanded_velocity_pub.publish(commanded_velocity_msg);

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





