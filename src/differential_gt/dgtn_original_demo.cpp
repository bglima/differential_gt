#include "ros/ros.h"
#include <differential_gt/cgt.h>
#include <differential_gt/ncgt.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> 
#include <geometry_msgs/WrenchStamped.h>

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
     // defining the ros_node. The third argument is the name of the node.
     ros::init(argc, argv, "diff_game_theory_node");
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

     // We define a constant reference to be followed by the robot.
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
  
     // Initialize the Non-Cooperative GT controller
     NonCoopGT ncgt(n_dofs,dt);

     // Set the System Parameters
     ncgt.setSysParams(Ac,Bc);

     // Get the System Parameters
     ncgt.getSysParams(Ac,Bc,Cc);

     // Print the System Parameters
     std::cout<<"SYSTEM PARAMETERS: \n";

     ROS_INFO_STREAM("Ac: \n" << Ac << "\n");
     ROS_INFO_STREAM("Bc: \n" << Bc << "\n");
     ROS_INFO_STREAM("Cc: \n" << Cc << "\n");   
  
  CoopGT cgt(n_dofs,dt);
  cgt.setSysParams(Ac,Bc);
  
  Eigen::VectorXd X=Eigen::VectorXd::Zero(2*n_dofs);
  Eigen::VectorXd dX=Eigen::VectorXd::Zero(2*n_dofs);

  cgt.setCurrentState(X);
  ncgt.setCurrentState(X);
  
  Eigen::MatrixXd Qhh; Qhh.resize(2,2); 
  Eigen::MatrixXd Qhr; Qhr.resize(2,2);
  Eigen::MatrixXd Qrr; Qrr.resize(2,2);
  Eigen::MatrixXd Qrh; Qrh.resize(2,2); 
  
  Qhh <<1,0,
       0,0.0001;
  Qhr <<0,0,
       0,0.0001;
       
  Qrr <<1,0,
       0,0.0001;
  Qrh <<0.0001,0,
       0,0;
  Eigen::MatrixXd Rh; Rh.resize(1,1); Rh<< .0005;
  Eigen::MatrixXd Rr; Rr.resize(1,1); Rr<< .0001;
  double alpha = 0.8;
  
  cgt.setAlpha(alpha);
  
  cgt.setCostsParams(Qhh,Qhr,Qrh,Qrr,Rh,Rr);
  
  Eigen::MatrixXd Qh; Qh.resize(2,2);
  Eigen::MatrixXd Qr; Qr.resize(2,2);
  cgt.getCostMatrices(Qh,Qr,Rh,Rr);
  
  ncgt.setCostsParams(Qh,Qr,Rh,Rr);
  
  ROS_INFO_STREAM("Qh: \n"<<Qh);
  ROS_INFO_STREAM("Qr: \n"<<Qr);
  ROS_INFO_STREAM("Rh: \n"<<Rh);
  ROS_INFO_STREAM("Rr: \n"<<Rr);
  
  
  cgt.computeCooperativeGains();
  
  ROS_INFO_STREAM("cgt: \n"<<cgt.getCooperativeGains());
  
  ncgt.computeNonCooperativeGains();
  
  Eigen::MatrixXd Kh,Kr;
  ncgt.getNonCooperativeGains(Kh,Kr);
  ROS_INFO_STREAM("Kh: \n"<<Kh);
  ROS_INFO_STREAM("Kr: \n"<<Kr);

  std::cin.get();
  
  Eigen::VectorXd rh = ref_h.segment(0,1);
  Eigen::VectorXd rr = ref_r.segment(0,1);
  cgt.setPosReference(rh,rr);
  ncgt.setPosReference(rh,rr);  

  for (int i = 0;i<10;i++)  
  {
    
    rh = ref_h.segment(i,1);
    rr = ref_r.segment(i,1);
    
    Eigen::VectorXd cgt_state = cgt.getCurrentState();
    cgt.step(cgt_state ,rh,rr);
    ROS_INFO_STREAM("cgt_state : "<<cgt_state .transpose());
    
    Eigen::VectorXd ncgt_state = ncgt.getCurrentState();
    ncgt.step(ncgt_state ,rh,rr);
    ROS_INFO_STREAM("ncgt_state : "<<ncgt_state .transpose());

//     cgt.setPosReference(rh,rr);
//     cgt.setCurrentState(X);
//     cgt.computeCooperativeGains();
//     Eigen::MatrixXd Kgt = cgt.getCooperativeGains();
//     Eigen::VectorXd control = cgt.computeControlInputs();
//     dX = Ac*X + Bc*control.segment(0,n_dofs) + Bc*control.segment(n_dofs,n_dofs);
//     X = X + dX*dt;
//     ROS_INFO_STREAM("control: " << control.transpose());
//     ROS_INFO_STREAM("state: " << X.transpose());

  }
  
  
  
  
  
  
  return 0;
}





