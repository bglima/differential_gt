#include "ros/ros.h"
#include <differential_gt/cgt.h>
#include <differential_gt/ncgt.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> 
#include <geometry_msgs/WrenchStamped.h>

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

int main(int argc, char **argv)
{
     ros::init(argc, argv, "coop_node"); // defining the ros_node. The third argument is the name of the node
     /*NodeHandle is the main access point to communications with the ROS system.
     The first Nodehandle constructed will fully initialize this node;
     the last NodeHandle destructed will close down the node.*/

     ros::NodeHandle n;
     
     int n_dofs=1;
     double dt = 0.01;
     
     std::vector<double> time = range(0.0, 6 * M_PI - dt, dt); // M_PI = 3.14159

     // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

     Eigen::VectorXd ref_h; ref_h.resize(time.size());
     Eigen::VectorXd ref_r; ref_r.resize(time.size());
     
     // We define a constant reference to be followed by the robot
     /* 
      We will have three intervals of 2*pi here
      (1) In the first interval, the robot reference will be zero.
      (2) In the second interval, the robot reference will be the same as the human.
      (3) In the last interval, the robot reference will be the reverse of the human
     */

     for (int i = 0; i<time.size(); i++)
     {
          ref_h(i) = 0.6*std::sin(time[i]);
          
          if (i <= time.size() / 3)
          ref_r(i) = 0;

          else if (i <= 2 * time.size() / 3)
          ref_r(i) = 0.6*std::sin(time[i]);

          else
          ref_r(i) = -0.6*std::sin(time[i]);
     }

     Eigen::MatrixXd Ac; Ac.resize(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Bc; Bc.resize(2*n_dofs,n_dofs);
     Eigen::MatrixXd Cc; Cc.resize(n_dofs,2*n_dofs);
  
     double m,c,k;
     // Initialize the impedance parameters in the original formulation
     
     m=10;
     k=0;
     c=25;
     
     // Initialize the linearized state space matrices
     Ac << 0, 1,
     -k/m, -c/m;
     
     Bc << 0,
     1/m;
     
     Cc << 1, 0;

     // Initialize the Cooperative GT controller
     CoopGT cgt(n_dofs,dt);

     // Set the System Parameters
     cgt.setSysParams(Ac,Bc);

     // Get the System Parameters
     cgt.getSysParams(Ac,Bc,Cc);    
     
     // Initialize the state-error-weight cost matrices 
     Eigen::MatrixXd Qhh; Qhh.resize(2,2); 
     Eigen::MatrixXd Qhr; Qhr.resize(2,2);
     Eigen::MatrixXd Qrr; Qrr.resize(2,2);
     Eigen::MatrixXd Qrh; Qrh.resize(2,2); 

     // Initialize the weigthed state-error-weight cost matrices 
     Eigen::MatrixXd Qh; Qh.resize(2,2);
     Eigen::MatrixXd Qr; Qr.resize(2,2);
     
     // Fill the state-error-weight cost matrices
  
     // Human state-error-weight cost component based on human references
     Qhh <<1,0,
       0,0.0001;

     // Human state-error-weight cost component based on robot references
     Qhr <<0,0,
       0,0.0001;

     // Robot state-error-weight cost component based on robot references
     Qrr <<1,0,
       0,0.0001;

     // Robot state-error-weight cost component based on human references
     Qrh <<0.0001,0,
       0,0;
     
     // Initialize the control-input cost matrices
     Eigen::MatrixXd Rh; Rh.resize(1,1); Rh << .0005;
     Eigen::MatrixXd Rr; Rr.resize(1,1); Rr << .0001;

     // Initialize the alpha value 
     double alpha = 0.75;

     // set the Alpha parameter
     cgt.setAlpha(alpha);

     // Set the CGT parameters
     cgt.setCostsParams(Qhh,Qhr,Qrh,Qrr,Rh,Rr);

     // Get Cost Matrices Qh, Qr, Rh, Rr
     cgt.getCostMatrices(Qh,Qr,Rh,Rr);

     // Initialize the Current State
     Eigen::VectorXd X = Eigen::VectorXd::Zero(2*n_dofs);
     Eigen::VectorXd dX = Eigen::VectorXd::Zero(2*n_dofs);

     // Set the Current State
     cgt.setCurrentState(X);

     // Get the Current State
     X = cgt.getCurrentState();

     // The following method computes the Cooperative gain Kgt
     cgt.computeCooperativeGains();

     // Initialize the gain Matrix K_gt and set the values   
     Eigen::MatrixXd Kgt = cgt.getCooperativeGains();
  
     // In here, we define a first positional reference to our controller

     // In Eigen::VectorXd, the .segment block operation can specify a
     // a block containing n elements, starting at position i following
     // this structure: vector.segment(i,n);

     Eigen::VectorXd rh = ref_h.segment(0,1);
     Eigen::VectorXd rr = ref_r.segment(0,1);
     cgt.setPosReference(rh,rr);  

     // Get the first weighted reference
     Eigen::VectorXd weighted_reference;
     weighted_reference = cgt.getReference();

     // Instantiate ROS state messages
     geometry_msgs::PoseStamped state_pose_msg;
     geometry_msgs::TwistStamped state_velocity_msg;
     geometry_msgs::PoseStamped human_reference_pose_msg;
     geometry_msgs::TwistStamped human_reference_velocity_msg;
     geometry_msgs::PoseStamped robot_reference_pose_msg;
     geometry_msgs::TwistStamped robot_reference_velocity_msg;
     geometry_msgs::PoseStamped weighted_reference_pose_msg;
     geometry_msgs::PoseStamped weighted_reference_velocity_msg;

     // Instantiate ROS control messages
    //  geometry_msgs::WrenchStamped optimal_control_robot_msg;
    //  geometry_msgs::WrenchStamped optimal_control_human_msg;
     geometry_msgs::WrenchStamped optimal_control_weighted_msg;

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
    //  ros::Publisher optimal_control_robot_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/robot", 1);
    //  ros::Publisher optimal_control_human_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/human", 1);
     ros::Publisher optimal_control_weighted_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/weighted", 1);

     // Create a ROS loop rate
     ros::Rate control_rate(1.0/dt);

     // Create a ROS time reference from the starting moment
     ros::Time starting_time = ros::Time::now();
     ros::Time seconds_from_start;

     // Create a control object store future optimal control inputs
     Eigen::VectorXd control;
     control = cgt.computeControlInputs();

     // Indexes initialization
     int reference_index = 0;
     long double current_time = 0;

     // // Learning something about the ros::Time
     // double secs = ros::Time::now().toNSec();

     // std::cout << "valore attuale della variabile secs:\n";
     // std::cout << secs << "\n";

     // ros::Duration d(0.5);
     // secs = d.toSec();

     // std::cout << "cosa succede dopo le righe precedenti \n";
     // std::cout << secs << "\n";

     // At this point, the initialization is complete.
     // The code asks for a key to continue the process
     ROS_INFO_STREAM("The controller is initialized. Press ENTER to start the demo.");
     std::cin.get();

     // Main loop
     while (ros::ok())
     {
          reference_index += 1;
          reference_index = (reference_index % time.size());
          ROS_INFO("Reference index is %d", reference_index);

          current_time += dt;

          rh = ref_h.segment(reference_index, 1);
          rr = ref_r.segment(reference_index, 1);

          // // Set the current positional reference
          cgt.setPosReference(rh, rr);

          // We need to update the state with the real robot data. In this case, 
          // we will update with the last known state.
          Eigen::VectorXd cgt_state = cgt.getCurrentState();
          cgt.setCurrentState(cgt_state);

          // This step variable assumes that the optimal control will be performed by
          // both human and robot. In the future, we will create a new step() method
          // that accepts a control input from one of the agents. We assume that:
          //   1 : reference/control human
          //   2 : reference/control robot
          //
          cgt.step(cgt_state, rh, rr);

          // We retrieve the optimal control inputs from before the state has been
          // performed. This command is performed also inside the ncgt.step(). The 
          // optimal control inputs are calculated based on the current state.
          cgt.getControlInput(control);

          ROS_INFO_STREAM("Control input: " << control.transpose());

          weighted_reference = cgt.getReference();

          ROS_INFO_STREAM("weighted_reference: " << weighted_reference.transpose());

          // Note that we print the state stored before the step has been done.
          // In other words, we print the previous state. 
          ROS_INFO_STREAM("cgt_state : "<< cgt_state .transpose());

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

          // optimal_control_human_msg.header.stamp = seconds_from_start;
          // optimal_control_robot_msg.header.stamp = seconds_from_start;

          optimal_control_weighted_msg.header.stamp = seconds_from_start;

          // Update the reference messages. They are positional references only.
          human_reference_pose_msg.pose.position.y = rh(0);
          human_reference_velocity_msg.twist.linear.y = 0;

          robot_reference_pose_msg.pose.position.y = rr(0);
          robot_reference_velocity_msg.twist.linear.y = 0;

          weighted_reference_pose_msg.pose.position.y = weighted_reference(0);

          // Update state pose message.
          // Since it is a unidimensional problem, we will use only the X positional axis. All the rest, leave as zero.
          state_pose_msg.pose.position.x = 0.65;
          state_pose_msg.pose.position.y = cgt_state(0);
          state_pose_msg.pose.position.z = 0.6;
          state_pose_msg.pose.orientation.x = 1;
          state_pose_msg.pose.orientation.y = 0;
          state_pose_msg.pose.orientation.z = 0;
          state_pose_msg.pose.orientation.w = 0;

          // Update state velocity message.
          // Since it is a unidimensional problem, we will use only the X positional axis. All the rest, leave as zero.
          state_velocity_msg.twist.linear.y = cgt_state(1);

          // Update the control messages.
          // optimal_control_human_msg.wrench.force.x = control(0);
          // optimal_control_robot_msg.wrench.force.x = control(1);
          optimal_control_weighted_msg.wrench.force.y = control(0);

          // Publish messages
          state_pose_pub.publish(state_pose_msg);
          state_velocity_pub.publish(state_velocity_msg);

          human_reference_pose_pub.publish(human_reference_pose_msg);
          human_reference_velocity_pub.publish(human_reference_velocity_msg);

          robot_reference_pose_pub.publish(robot_reference_pose_msg);
          robot_reference_velocity_pub.publish(robot_reference_velocity_msg);

          weighted_reference_pose_pub.publish(weighted_reference_pose_msg);
          weighted_reference_velocity_pub.publish(weighted_reference_velocity_msg);

          // optimal_control_human_pub.publish(optimal_control_human_msg);
          // optimal_control_robot_pub.publish(optimal_control_robot_msg);

          optimal_control_weighted_pub.publish(optimal_control_weighted_msg);

          // Synchronize
          control_rate.sleep();
     }

  return 0;
}





