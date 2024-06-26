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

void alphaCallback(const std_msgs::Float32::ConstPtr& msg)
{
     ROS_INFO("I heard: [%f]", msg->data);
     alpha = msg->data;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "diff_game_theory_node"); // defining the ros_node. The third argument is the name of the node
     /*NodeHandle is the main access point to communications with the ROS system.
     The first Nodehandle constructed will fully initialize this node;
     the last NodeHandle destructed will close down the node.*/

     ros::NodeHandle n;

     ros::AsyncSpinner spinner(4);
     spinner.start();
     
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
          ref_h(i) = 0.5*std::sin(time[i]);
          
          if (i <= time.size() / 3)
          ref_r(i) = 0;

          else if (i <= 2 * time.size() / 3)
          ref_r(i) = 0.5*std::sin(time[i]);

          else
          ref_r(i) = -0.5*std::sin(time[i]);
     }

     Eigen::MatrixXd Ac; Ac.resize(2*n_dofs,2*n_dofs);
     Eigen::MatrixXd Bc; Bc.resize(2*n_dofs,n_dofs);
     Eigen::MatrixXd Cc; Cc.resize(n_dofs,2*n_dofs);
  
     double m,c,k;
     // Initialize the impedance parameters in the original formulation
     
     m = 10; // [Kg]
     k = 0; // [N/m]
     c = 25; // [Ns/m]
     
     // Initialize the linearized state space matrices
     Ac << 0, 1,
     -k/m, -c/m;
     
     Bc << 0,
     1/m;
     
     Cc << 1, 0;

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
     Eigen::MatrixXd Rh; Rh.resize(n_dofs,n_dofs); Rh << .0005;
     Eigen::MatrixXd Rr; Rr.resize(n_dofs,n_dofs); Rr << .0001;

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
     Eigen::VectorXd X = Eigen::VectorXd::Zero(2*n_dofs);
     Eigen::VectorXd dX = Eigen::VectorXd::Zero(2*n_dofs);

     // Set the Cooperative current State
     cgt.setCurrentState(X);

     // Set the Non-cooperative current state
     ncgt.setCurrentState(X);

     // Get the Cooperative current state
     X = cgt.getCurrentState();

     // Get the Non-cooperative current state
     X = ncgt.getCurrentState();
  
     // The following method computes the Cooperative gain Kgt
     cgt.computeCooperativeGains();

     // The following method computes the Non-Cooperative gains Kh and Kr
     ncgt.computeNonCooperativeGains();

     // Initialize and the Cooperative gain   
     Eigen::MatrixXd Kgt = cgt.getCooperativeGains();

     // Get the Non-Cooperative gains
     Eigen::MatrixXd Kh,Kr;
     ncgt.getNonCooperativeGains(Kh,Kr);
  
     /* IN HERE, WE DEFINE A FISRT POSITIONAL REFERENCE TO OUR CONTROLLER */ 

     // In Eigen::VectorXd, the .segment block operation can specify a
     // a block containing n elements, starting at position i following
     // this structure: vector.segment(i,n);

     Eigen::VectorXd rh = ref_h.segment(0,1);
     Eigen::VectorXd rr = ref_r.segment(0,1);

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
     ros::Publisher optimal_control_robot_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/robot", 1);
     ros::Publisher optimal_control_human_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/human", 1);
     ros::Publisher optimal_control_weighted_pub = n.advertise<geometry_msgs::WrenchStamped>("/control/weighted", 1);

     // Create a ROS loop rate
     ros::Rate control_rate(1.0/dt);

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
          // ROS_INFO("Reference index is %d", reference_index);

          current_time += dt;

          rh = ref_h.segment(reference_index, 1);
          rr = ref_r.segment(reference_index, 1);

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

          optimal_control_weighted_msg.header.stamp = seconds_from_start;

          // Update the reference messages. They are positional references only.
          human_reference_pose_msg.pose.position.y = rh(0);
          human_reference_velocity_msg.twist.linear.y = 0;

          robot_reference_pose_msg.pose.position.y = rr(0);
          robot_reference_velocity_msg.twist.linear.y = 0;

          weighted_reference_pose_msg.pose.position.y = weighted_reference(0);
          weighted_reference_velocity_msg.twist.linear.y = 0;

          // Update state pose message.
          // Since it is a unidimensional problem, we will use only the X positional axis. All the rest, leave as zero.
          state_pose_msg.pose.position.x = 0.35;
          if (alpha >= 0.5)
               state_pose_msg.pose.position.y = cgt_state(0);
          else if (alpha < 0.5)
               state_pose_msg.pose.position.y = ncgt_state(0);
                    
          state_pose_msg.pose.position.z = 0.5;
          state_pose_msg.pose.orientation.x = 1;
          state_pose_msg.pose.orientation.y = 0;
          state_pose_msg.pose.orientation.z = 0;
          state_pose_msg.pose.orientation.w = 0;

          // Update state velocity message.
          // Since it is a unidimensional problem, we will use only the Y positional axis. All the rest, leave as zero.
          if (alpha >= 0.5)
               state_velocity_msg.twist.linear.y = cgt_state(1);
          else if (alpha < 0.5)
               state_velocity_msg.twist.linear.y = ncgt_state(1);

          // Update the control messages.
          optimal_control_human_msg.wrench.force.y = non_coop_control(0);
          optimal_control_robot_msg.wrench.force.y = non_coop_control(1);
          optimal_control_weighted_msg.wrench.force.y = coop_control(0);

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

          optimal_control_weighted_pub.publish(optimal_control_weighted_msg);

          // Synchronize
          control_rate.sleep();
     }
     spinner.stop();
  return 0;
}





