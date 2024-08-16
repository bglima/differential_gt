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
double alpha = 0.001;

// Definition of the human and robot references that will be assigned through the subscription.
geometry_msgs::PoseStamped ref_h;
geometry_msgs::PoseStamped ref_r;

/* n_dofs definition. In this case, assuming that the orientation of the panda gripper is always in the same condition, 
it is not relevant to control the three rotational components (roll, pitch, yaw). Thus, our intention is to control the 
three translational components only. Hence, n_dofs = 3. 
We read the three rotational components (in quaternion form) from the current pose of the robot end-effector in the 
callback function defined below. */
int n_dofs = 3;

// Initialize the Current State
Eigen::VectorXd Z = Eigen::VectorXd::Zero(2*n_dofs);
Eigen::VectorXd dZ = Eigen::VectorXd::Zero(2*n_dofs);

// Indicates if the first initial robot pose is received.
bool initial_robot_state_ok = false;

// Callback function used for receiving the alpha parameter from another node.
void alphaCallback(const std_msgs::Float32::ConstPtr& msg)
{
     alpha = msg->data;
}

// Callback function used for receiving the human reference from another node.
void human_refCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     // In this case, the rotational components are not updated because of the above consideration
     ref_h.pose.position.x = msg->pose.position.x;
     ref_h.pose.position.y = msg->pose.position.y;
     ref_h.pose.position.z = msg->pose.position.z;

}

// Callback function used for receiving the robot reference from another node.
void robot_refCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     // In this case, the rotational components are not updated because of the above consideration
     ref_r.pose.position.x = msg->pose.position.x;
     ref_r.pose.position.y = msg->pose.position.y;
     ref_r.pose.position.z = msg->pose.position.z;

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
     Eigen::Vector3d euler_angles_RPY = rot_mat_quaternion.toRotationMatrix().eulerAngles(0,1,2);
     // Assignment of the relevant parameters to the state vector
     Z(0) = msg->O_T_EE[12];
     Z(1) = msg->O_T_EE[13];
     Z(2) = msg->O_T_EE[14];

     // Fill initial references with the current robot state. The orientation is the one provided by franka_state.
     // Human reference
     ref_h.pose.position.x = Z(0);
     ref_h.pose.position.y = Z(1);
     ref_h.pose.position.z = Z(2);
     ref_h.pose.orientation.x = rot_mat_quaternion.x();
     ref_h.pose.orientation.y = rot_mat_quaternion.y();
     ref_h.pose.orientation.z = rot_mat_quaternion.z();
     ref_h.pose.orientation.w = rot_mat_quaternion.w();
     // Robot reference
     ref_r.pose.position.x = Z(0);
     ref_r.pose.position.y = Z(1);
     ref_r.pose.position.z = Z(2);
     ref_r.pose.orientation.x = rot_mat_quaternion.x();
     ref_r.pose.orientation.y = rot_mat_quaternion.y();
     ref_r.pose.orientation.z = rot_mat_quaternion.z();
     ref_r.pose.orientation.w = rot_mat_quaternion.w();

     // Printing robot first pose condition, first ref_h and first ref_r
     std::cout << "ref_h: \n" << ref_h << "\n";
     std::cout << "ref_r: \n" << ref_r << "\n";
     std::cout << "Z: \n" << Z << "\n";
     
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

     // WE HAVE TO CHANGE THE SUBSCRIBER BUFFER IN ORDER TO WORK IN A PROPER WAY

     // Subscribing to a topic called '/alpha' so that we can have the alpha parameter coming from an external node. 
     ros::Subscriber alpha_sub = n.subscribe("/alpha", 30, alphaCallback);
     // Subscribing to a topic called '/human_ref' so that we can have the human reference coming from an external node.
     ros::Subscriber human_ref_sub = n.subscribe("/human_ref", 30, human_refCallback);
     // Subscribing to a topic called '/robot_ref' so that we can have the robot reference coming from an external node.
     ros::Subscriber robot_ref_sub = n.subscribe("/robot_ref", 30, robot_refCallback);
     // Subscribing to a topic called '/franka_state_controller/franka_states' so that we can have the actual state of the robot.
     ros::Subscriber current_robot_state_sub = n.subscribe("/franka_state_controller/franka_states", 30, current_robot_stateCallback); 
     
     while(!initial_robot_state_ok)
     {
          ROS_INFO("waiting for an initial robot pose");
          ros::Duration(5).sleep();
     }

     // In this case, the n_dofs variable is extended to 6. The parameters that are modified come from the 
     // gt_traj_arbitration package of Paolo Franceschi's repository (https://github.com/paolofrance/gt_traj_arbitration)
     double rate = 30;
     double dt = 1.0/rate;

     // Defining the Identity and Null Matrices
     Eigen::MatrixXd O; O.resize(n_dofs, n_dofs); O.setZero();
     Eigen::MatrixXd I; I.resize(n_dofs, n_dofs); I.setIdentity();

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
     D << 25*I; // The previous parameter was 100*I

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
     
     // Initialize the Cooperative GT controller state-error-weight cost matrices. 
     // They are also used in the Non-cooperative GT controller as it is written in Franceschi's paper.
     Eigen::MatrixXd Qhh; Qhh.resize(2*n_dofs,2*n_dofs); 
     Eigen::MatrixXd Qhr; Qhr.resize(2*n_dofs,2*n_dofs); 
     Eigen::MatrixXd Qrr; Qrr.resize(2*n_dofs,2*n_dofs); 
     Eigen::MatrixXd Qrh; Qrh.resize(2*n_dofs,2*n_dofs); 

     // Initialize the Cooperative GT controller weighted state-error-weight cost matrices. 
     // They are also used in the Non-cooperative GT controller as it is written in Franceschi's paper.
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
     Eigen::MatrixXd Rr; Rr.resize(n_dofs,n_dofs); Rr << 0.0001*I;

     cgt.setAlpha(alpha);

     /* SET THE DIFF GAME THEORY PARAMETERS*/

     // Set the Cooperative cost parameters
     cgt.setCostsParams(Qhh,Qhr,Qrh,Qrr,Rh,Rr);

     // Get Cost Matrices Qh, Qr, Rh, Rr. These are the matrices that will be passed in the Non-cooperative case for the reasoning commented above.
     cgt.getCostMatrices(Qh,Qr,Rh,Rr);

     // // Print the Cooperative cost matrices.
     // std::cout<< "COST PARAMETERS COOPERATIVE CASE: \n";
     
     // ROS_INFO_STREAM("Qhh: \n" << Qhh << "\n");
     // ROS_INFO_STREAM("Qhr: \n" << Qhr << "\n");
     // ROS_INFO_STREAM("Qrh: \n" << Qrh << "\n");
     // ROS_INFO_STREAM("Qrr: \n" << Qrr << "\n");
     // ROS_INFO_STREAM("Qh: \n" << Qh << "\n");
     // ROS_INFO_STREAM("Qr: \n" << Qr << "\n");
     // ROS_INFO_STREAM("Rh: \n" << Rh << "\n");
     // ROS_INFO_STREAM("Rr: \n" << Rr << "\n");

     // Set the Non-cooperative cost parameters, based on the ones got from the previous line.
     ncgt.setCostsParams(Qh,Qr,Rh,Rr);

     // Get the Non-cooperative matrices.
     ncgt.getCostMatrices(Qh,Qr,Rh,Rr);

     // // Print the Non-cooperative cost matrices.
     // std::cout<< "COST PARAMETERS NON-COOPERATIVE CASE: \n";
     
     // ROS_INFO_STREAM("Qh: \n" << Qh << "\n");
     // ROS_INFO_STREAM("Qr: \n" << Qr << "\n");
     // ROS_INFO_STREAM("Rh: \n" << Rh << "\n");
     // ROS_INFO_STREAM("Rr: \n" << Rr << "\n");

     /* CURRENT STATE*/

     // Set the Cooperative current State
     cgt.setCurrentState(Z);

     // Set the Non-cooperative current state
     ncgt.setCurrentState(Z);

     // Get the Cooperative current state
     Z = cgt.getCurrentState();

     // Get the Non-cooperative current state
     Z = ncgt.getCurrentState();

     /* GAIN MATRICES*/
  
     // The following method computes the Cooperative gain Kgt
     cgt.computeCooperativeGains();

     // The following method computes the Non-Cooperative gains Kh and Kr
     ncgt.computeNonCooperativeGains();

     // Initialize and the Cooperative gain   
     Eigen::MatrixXd Kgt = cgt.getCooperativeGains();

     // Get the Non-Cooperative gains
     Eigen::MatrixXd Kh,Kr;
     ncgt.getNonCooperativeGains(Kh,Kr);

     // ROS_INFO_STREAM("Kgt: \n" << Kgt << "\n");
     // ROS_INFO_STREAM("Kh: \n" << Kh << "\n");
     // ROS_INFO_STREAM("Kr: \n" << Kr << "\n");
  
     /* IN HERE, WE DEFINE A FIRST POSITIONAL REFERENCE TO OUR CONTROLLER */

     Eigen::VectorXd rh; rh.resize(n_dofs);
     rh << ref_h.pose.position.x, ref_h.pose.position.y, ref_h.pose.position.z;
     Eigen::VectorXd rr; rr.resize(n_dofs);
     rr << ref_r.pose.position.x, ref_r.pose.position.y, ref_r.pose.position.z;

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

     ros::Publisher commanded_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 30);
     ros::Publisher commanded_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/state/velocity", 30);

     ros::Publisher human_reference_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/reference/human/pose", 30);
     ros::Publisher human_reference_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/reference/human/velocity", 30);

     ros::Publisher robot_reference_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/reference/robot/pose", 30);
     ros::Publisher robot_reference_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/reference/robot/velocity", 30);

     ros::Publisher weighted_reference_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/reference/weighted/pose", 30);
     ros::Publisher weighted_reference_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/reference/weighted/velocity", 30);

     ros::Publisher optimal_control_robot_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/robot", 30);
     ros::Publisher optimal_control_human_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/human", 30);

     ros::Publisher optimal_control_human_weighted_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/human_weighted", 30);
     ros::Publisher optimal_control_robot_weighted_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/robot_weighted", 30);

     // Create a ROS loop rate
     ros::Rate control_rate(rate);

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

     // Index initialization
     long double current_time = 0;

     ROS_INFO_STREAM("The controller is initialized. The demo starts now.");

     // Main loop
     while (ros::ok())
     {
          rh << ref_h.pose.position.x, ref_h.pose.position.y, ref_h.pose.position.z;
          rr << ref_r.pose.position.x, ref_r.pose.position.y, ref_r.pose.position.z;
          
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

          current_time += dt;
          
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
     
          // Get Cooperative and Non-cooperative references.
          weighted_reference = cgt.getReference();
          ncgt.getReference(rh,rr);

          // ROS_INFO_STREAM("Coop control input: " << coop_control.transpose());
          // ROS_INFO_STREAM("Non-coop Control input: " << non_coop_control.transpose());
          // ROS_INFO_STREAM("weighted_reference: " << weighted_reference.transpose());
          // ROS_INFO_STREAM("human reference: " << rh.transpose());
          // ROS_INFO_STREAM("robot reference: " << rr.transpose());

          // Note that we print the state stored before the step has been done.
          // In other words, we print the previous state. 
          // ROS_INFO_STREAM("cgt_state: " << cgt_state.transpose());
          // ROS_INFO_STREAM("ncgt_state: " << ncgt_state.transpose());

          // Update time
          seconds_from_start = ros::Time(current_time);

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
          human_reference_pose_msg.pose.orientation.x = ref_h.pose.orientation.x;
          human_reference_pose_msg.pose.orientation.y = ref_h.pose.orientation.y;
          human_reference_pose_msg.pose.orientation.z = ref_h.pose.orientation.z;
          human_reference_pose_msg.pose.orientation.w = ref_h.pose.orientation.w;

          // Robot reference positions
          robot_reference_pose_msg.pose.position.x = rr(0);
          robot_reference_pose_msg.pose.position.y = rr(1);
          robot_reference_pose_msg.pose.position.z = rr(2);

          // Robot reference orientations
          robot_reference_pose_msg.pose.orientation.x = ref_r.pose.orientation.x;
          robot_reference_pose_msg.pose.orientation.y = ref_r.pose.orientation.y;
          robot_reference_pose_msg.pose.orientation.z = ref_r.pose.orientation.z;
          robot_reference_pose_msg.pose.orientation.w = ref_r.pose.orientation.w;

          // Weighted reference positions
          weighted_reference_pose_msg.pose.position.x = weighted_reference(0);
          weighted_reference_pose_msg.pose.position.y = weighted_reference(1);
          weighted_reference_pose_msg.pose.position.z = weighted_reference(2);

          // Weighted reference orientations
          weighted_reference_pose_msg.pose.orientation.x = ref_h.pose.orientation.x;
          weighted_reference_pose_msg.pose.orientation.y = ref_h.pose.orientation.y;
          weighted_reference_pose_msg.pose.orientation.z = ref_h.pose.orientation.z;
          weighted_reference_pose_msg.pose.orientation.w = ref_h.pose.orientation.w;


          // Update state pose message.
          if (alpha >= 0.5)
          {
               // commanded positions
               commanded_pose_msg.pose.position.x = cgt_state(0);
               commanded_pose_msg.pose.position.y = cgt_state(1);
               commanded_pose_msg.pose.position.z = cgt_state(2);
               // commanded orientations set equal to ref_h 
               commanded_pose_msg.pose.orientation.x = ref_h.pose.orientation.x;
               commanded_pose_msg.pose.orientation.y = ref_h.pose.orientation.y;
               commanded_pose_msg.pose.orientation.z = ref_h.pose.orientation.z;
               commanded_pose_msg.pose.orientation.w = ref_h.pose.orientation.w;
          }

          else if (alpha < 0.5)
          {
               // commanded positions
               commanded_pose_msg.pose.position.x = ncgt_state(0);
               commanded_pose_msg.pose.position.y = ncgt_state(1);
               commanded_pose_msg.pose.position.z = ncgt_state(2);
               // commanded orientations set equal to ref_r
               commanded_pose_msg.pose.orientation.x = ref_r.pose.orientation.x;
               commanded_pose_msg.pose.orientation.y = ref_r.pose.orientation.y;
               commanded_pose_msg.pose.orientation.z = ref_r.pose.orientation.z;
               commanded_pose_msg.pose.orientation.w = ref_r.pose.orientation.w;
          }

          // Update state velocity message.
          if (alpha >= 0.5)
          {
               commanded_velocity_msg.twist.linear.x = cgt_state(3);
               commanded_velocity_msg.twist.linear.y = cgt_state(4);
               commanded_velocity_msg.twist.linear.y = cgt_state(5);
               // we don't want any angular velocity of the end-effector since it has to be fixed in position.
               commanded_velocity_msg.twist.angular.x = 0;
               commanded_velocity_msg.twist.angular.y = 0;
               commanded_velocity_msg.twist.angular.z = 0;
          }
          else if (alpha < 0.5)
          {
               commanded_velocity_msg.twist.linear.x = ncgt_state(6);
               commanded_velocity_msg.twist.linear.y = ncgt_state(7);
               commanded_velocity_msg.twist.linear.y = ncgt_state(8);
               // we don't want any angular velocity of the end-effector since it has to be fixed in position.
               commanded_velocity_msg.twist.angular.x = 0;
               commanded_velocity_msg.twist.angular.y = 0;
               commanded_velocity_msg.twist.angular.z = 0;
          }

          // Update the control messages.
          optimal_control_human_msg.wrench.force.x = non_coop_control(0);
          optimal_control_human_msg.wrench.force.y = non_coop_control(1);
          optimal_control_human_msg.wrench.force.z = non_coop_control(2);
          optimal_control_human_msg.wrench.torque.x = 0;
          optimal_control_human_msg.wrench.torque.y = 0;
          optimal_control_human_msg.wrench.torque.z = 0;

          optimal_control_robot_msg.wrench.force.x = non_coop_control(3);
          optimal_control_robot_msg.wrench.force.y = non_coop_control(4);
          optimal_control_robot_msg.wrench.force.z = non_coop_control(5);
          optimal_control_robot_msg.wrench.torque.x = 0;
          optimal_control_robot_msg.wrench.torque.y = 0;
          optimal_control_robot_msg.wrench.torque.z = 0;

          optimal_control_human_weighted_msg.wrench.force.x = coop_control(0);
          optimal_control_human_weighted_msg.wrench.force.y = coop_control(1);
          optimal_control_human_weighted_msg.wrench.force.z = coop_control(2);
          optimal_control_human_weighted_msg.wrench.torque.x = 0;
          optimal_control_human_weighted_msg.wrench.torque.y = 0;
          optimal_control_human_weighted_msg.wrench.torque.z = 0;

          optimal_control_robot_weighted_msg.wrench.force.x = coop_control(3);
          optimal_control_robot_weighted_msg.wrench.force.y = coop_control(4);
          optimal_control_robot_weighted_msg.wrench.force.z = coop_control(5);
          optimal_control_robot_weighted_msg.wrench.torque.x = 0;
          optimal_control_robot_weighted_msg.wrench.torque.y = 0;
          optimal_control_robot_weighted_msg.wrench.torque.z = 0;

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





