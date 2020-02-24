#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include "tcc/in_loop_cmd_generator.h"
#include "tcc/geometry_math_type.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tcc/ParamConfig.h>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Sparse>
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include "ooqp_eigen_interface/ooqpei_gtest_eigen.hpp"

typedef Eigen::Triplet<double> Trip;
ros::Publisher rpyt_command_pub;
ros::Publisher mpc_sim_odom;
fullstate_t cmd_, current_;

void CtrloopCallback(const ros::TimerEvent&);
void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void Paramcallback(tcc::ParamConfig &config, uint32_t level);

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3);
Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current);
Eigen::Vector3f mpccontrol();

Eigen::Vector3f CtrlOmega(1.0, 1.0, 1.8); //norminal natural frequency
Eigen::Vector3f CtrlEpsilon(1, 1, 1); //tuning parameter
Eigen::Vector3f CtrlZita(1, 1, 1.1); //damping ratio
Eigen::Vector3f PosErrorAccumulated_(0, 0, 0);
Eigen::Vector3f k_I_(0.02, 0.02, 0.15);
mppi_control::InLoopCmdGen InLoopCmdGen_(0.4);
float posErrAccLimit_ = 10.0, acc_xy_limit_=2.0;
float k_p_yaw_=0.2, k_I_yaw_=0.2;
float yawErrorAccum_ = 0, yawErrorAccumLim_ = 3.1415927;
std::string sim_type_;
float thrust_;
//for mpc
double cost_g_ = 1.0, cost_h_ = 0.50, cost_v_ = 0.50;
double T_s = 0.1;
int horizon_N = 20;
Eigen::MatrixXd z_reference_(3*200, 1);
Eigen::VectorXd sim_state_ = Eigen::MatrixXd::Constant(6, 1, 0);
bool mpc_sim_;

int main(int argc, char** argv){
  ros::init(argc, argv, "tcc");

  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<tcc::ParamConfig> server;
  dynamic_reconfigure::Server<tcc::ParamConfig>::CallbackType externalfunction;
  externalfunction = boost::bind(&Paramcallback, _1, _2);
  server.setCallback(externalfunction);
  if (!nh.getParam("sim_type", sim_type_)){
    ROS_WARN("Don't have sim type parameter. Exiting");
    exit(-1);
  }
  nh.getParam("mpc_sim", mpc_sim_);

  if(sim_type_!="rotors" && sim_type_!="dji" && sim_type_!="none"){
    ROS_WARN("not good, don't know in simulation or real flight");
    exit(-1);
  }
  if (sim_type_!="rotors"){
    rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
                                      "/firefly/command/roll_pitch_yawrate_thrust1", 50);    
  } else {
    rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
                                      "/firefly/command/roll_pitch_yawrate_thrust", 50);        
  }

  mpc_sim_odom = nh.advertise<nav_msgs::Odometry>("mpc_sim_odom", 50);
  ros::Subscriber trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                                   "/firefly/command/trajectory", 10, trajectory_cb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 10, odom_cb);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/10), CtrloopCallback);

  //mpccontrol();
  ros::spin();
  //ros::spinOnce();
  return 0;
}

void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  cmd_.timestamp = ros::Time::now();
  geoVec3toEigenVec3(msg->points[0].transforms[0].translation, cmd_.pos);
  geoVec3toEigenVec3(msg->points[0].velocities[0].linear, cmd_.vel);
  geoVec3toEigenVec3(msg->points[0].accelerations[0].linear, cmd_.acc);
  Eigen::Quaternion<float> cmd_Quat(msg->points[0].transforms[0].rotation.w, 
                                    msg->points[0].transforms[0].rotation.x, 
                                    msg->points[0].transforms[0].rotation.y, 
                                    msg->points[0].transforms[0].rotation.z);
  get_dcm_from_q(cmd_.R, cmd_Quat);
  //std::cout<<"trarget pos x: "<<cmd_.pos(0)<<"y: "<<cmd_.pos(1)<<"z: "<<cmd_.pos(2)<<"\n";
  int _n = msg->points.size();
  for (int i=0; i<_n; i++){
    z_reference_(i*3+0, 0) = msg->points[i].transforms[0].translation.x;
    z_reference_(i*3+1, 0) = msg->points[i].transforms[0].translation.y;
    z_reference_(i*3+2, 0) = msg->points[i].transforms[0].translation.z;
  }
  //std::cout<<"z reference is"<<z_reference_<<"\n";
  ROS_INFO_ONCE("Got first trajectory message!");

}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_.timestamp = ros::Time::now();
  geoPt3toEigenVec3(msg->pose.pose.position, current_.pos);
  if (sim_type_!="rotors"){ //for real flight or using DJI simulator
    geoVec3toEigenVec3(msg->twist.twist.linear, current_.vel);
  } else { //for simulation with ROTORS
    Eigen::Quaternionf orientation_W_B(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3f velocity_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3f velocity_world = orientation_W_B *velocity_body;    
    current_.vel = velocity_world;
  }
  if (sim_type_=="none"){ //for real flight using dji
    tf::Quaternion q0(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Quaternion q1 = tf::createQuaternionFromRPY(3.1415927, 0, 0); //transform from vins imu frame to FLU frame
    tf::Quaternion qf = q0*q1;
    Eigen::Quaternion<float> current_Quat(qf.w(), qf.x(), qf.y(), qf.z());
    get_dcm_from_q(current_.R, current_Quat);
  } else { //for simulation with ROTORS or DJI simulator
    Eigen::Quaternion<float> current_Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);    
    get_dcm_from_q(current_.R, current_Quat);
  }
  ROS_INFO_ONCE("Got first odom message!");

}

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3)
{
  eigenVec3(0) = geoVector3.x;
  eigenVec3(1) = geoVector3.y;
  eigenVec3(2) = geoVector3.z;
}

void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3)
{
  eigenVec3(0) = geoPt3.x;
  eigenVec3(1) = geoPt3.y;
  eigenVec3(2) = geoPt3.z;
}
// void geoQuatoEigenQua (geometry_msgs::Quaternion geoQua, Eigen::Quaternion& eigenQua)
// {
//   eigenQua = Quaternion(geoQua.w, geoQua.x, geoQua.y, geoQua.z);
// }

Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current)
{ 
  PosErrorAccumulated_ += (cmd.pos - current.pos);
  for (int i =0; i<=2; i++){
    if (PosErrorAccumulated_(i)>posErrAccLimit_) PosErrorAccumulated_(i) = posErrAccLimit_;
    else if (PosErrorAccumulated_(i)<-posErrAccLimit_) PosErrorAccumulated_(i) = -posErrAccLimit_;
  }
  Eigen::Vector3f tarAcc;
  tarAcc(0) = pow(CtrlOmega(0)/CtrlEpsilon(0), 2)*(cmd.pos(0)-current.pos(0)) +
             2*CtrlZita(0)*CtrlOmega(0)/CtrlEpsilon(0)*(cmd.vel(0)-current.vel(0)) + 
             cmd.acc(0) + k_I_(0)*PosErrorAccumulated_(0);
  tarAcc(1) = pow(CtrlOmega(1)/CtrlEpsilon(1), 2)*(cmd.pos(1)-current.pos(1)) +
             2*CtrlZita(1)*CtrlOmega(1)/CtrlEpsilon(1)*(cmd.vel(1)-current.vel(1)) + 
             cmd.acc(1) + k_I_(1)*PosErrorAccumulated_(1);
  tarAcc(2) = pow(CtrlOmega(2)/CtrlEpsilon(2), 2)*(cmd.pos(2)-current.pos(2)) +
             2*CtrlZita(2)*CtrlOmega(2)/CtrlEpsilon(2)*(cmd.vel(2)-current.vel(2)) + 
             cmd.acc(2) +  + k_I_(2)*PosErrorAccumulated_(2) + ONE_G;

  if (tarAcc(0) >= acc_xy_limit_) tarAcc(0) = acc_xy_limit_;
  else if (tarAcc(0) < -acc_xy_limit_) tarAcc(0) = -acc_xy_limit_;
  if (tarAcc(1) >= acc_xy_limit_) tarAcc(1) = acc_xy_limit_;
  else if (tarAcc(1) < -acc_xy_limit_) tarAcc(1) = -acc_xy_limit_;
  if (tarAcc(2) >= 2.0*ONE_G) tarAcc(2) = 2.0*ONE_G;
  else if (tarAcc(2) < 0.3*ONE_G) tarAcc(2) = 0.3*ONE_G;
  std::cout<<"trarget acc x: "<<tarAcc(0)<<"y: "<<tarAcc(1)<<"z: "<<tarAcc(2)<<"\n";

  return tarAcc;
}

void CtrloopCallback(const ros::TimerEvent&)
{
  if (fabs((ros::Time::now() - cmd_.timestamp).toSec()) < 0.05 && 
    (ros::Time::now()-current_.timestamp).toSec()<=0.11){
    //do control
    ROS_INFO_ONCE("started control!");
    //Eigen::Vector3f tarAcc_ = prtcontrol(cmd_, current_);
    Eigen::Vector3f tarAcc_ = mpccontrol();
    /*enu to ned */
    Eigen::Vector3f RefAtti_ned, tarAtti_ned, currentAtti_ned;
    Eigen::Vector3f tarAcc_ned_;
    tarAcc_ned_(0) = tarAcc_(0);
    tarAcc_ned_(1) = -tarAcc_(1);
    tarAcc_ned_(2) = -tarAcc_(2);
    Eigen::Matrix3f cmd_R_ned, current_R_ned;
    cmd_R_ned = cmd_.R;
    cmd_R_ned(0,1) = -cmd_.R(0,1);
    cmd_R_ned(0,2) = -cmd_.R(0,2);
    cmd_R_ned(1,0) = -cmd_.R(1,0);
    cmd_R_ned(2,0) = -cmd_.R(2,0);    
    current_R_ned = current_.R;
    current_R_ned(0,1) = -current_.R(0,1);
    current_R_ned(0,2) = -current_.R(0,2);
    current_R_ned(1,0) = -current_.R(1,0);
    current_R_ned(2,0) = -current_.R(2,0);

    get_euler_from_R<float>(RefAtti_ned, cmd_R_ned);
    get_euler_from_R(currentAtti_ned, current_R_ned);
    //std::cout<<"current attitide roll: "<<currentAtti_ned(0)/3.14159*180<<"pitch: "<<currentAtti_ned(1)/3.14159*180<<"yaw: "<<currentAtti_ned(2)/3.14159*180<<"\n";
    mppi_control::InLoopCmdGen::drone_cmd_t in_loop_cmd = InLoopCmdGen_.cal_R_T(tarAcc_ned_, current_R_ned, currentAtti_ned(2));
    get_euler_from_R(tarAtti_ned, in_loop_cmd.R);
    //std::cout<<"command attitide roll: "<<tarAtti_ned(0)/3.14159*180<<"pitch: "<<tarAtti_ned(1)/3.14159*180<<"yaw: "<<tarAtti_ned(2)/3.14159*180<<"\n";
    std::cout<<"command thrust: "<<in_loop_cmd.T<<"\n";
    mav_msgs::RollPitchYawrateThrust rpyrt_msg;
    rpyrt_msg.roll = tarAtti_ned(0);
    rpyrt_msg.pitch = -tarAtti_ned(1);
    yawErrorAccum_ += (-RefAtti_ned(2) + currentAtti_ned(2));
    if (yawErrorAccum_>yawErrorAccumLim_) yawErrorAccum_ = yawErrorAccumLim_;
    else if (yawErrorAccum_<-yawErrorAccumLim_) yawErrorAccum_ = -yawErrorAccumLim_;

    rpyrt_msg.yaw_rate = wrapPi(-RefAtti_ned(2) + currentAtti_ned(2)) * k_p_yaw_ + yawErrorAccum_*k_I_yaw_;
    rpyrt_msg.thrust.x = 0;
    rpyrt_msg.thrust.y = 0;
    if (sim_type_=="rotors"){  //for simulation with ROTORS only
      rpyrt_msg.thrust.z = in_loop_cmd.T*43.75;      
    } else {
      rpyrt_msg.thrust.z = in_loop_cmd.T;
    }
    rpyt_command_pub.publish(rpyrt_msg);

  } else if(mpc_sim_){
    mpccontrol();
  } else ROS_INFO_ONCE("ref trajectory lagging behind odom time!");
}


void Paramcallback(tcc::ParamConfig &config, uint32_t level)
{

    CtrlOmega(0) = config.CtrlOmega_xy;
    CtrlOmega(1) = config.CtrlOmega_xy;
    CtrlOmega(2) = config.CtrlOmega_z;

    posErrAccLimit_ = config.PosErrorAccumulatedLimit_xyz;
    k_I_(0) = config.Pos_ki_xy;
    k_I_(1) = config.Pos_ki_xy;
    k_I_(2) = config.Pos_ki_z;
    k_p_yaw_ = config.k_p_yaw_;
    k_I_yaw_ = config.k_I_yaw_;
    cost_h_ = config.mpc_accel_weight_;
    thrust_ = config.thrust_;
    cost_v_ =  config.mpc_vel_weight_;
    yawErrorAccumLim_ = config.yawErrorAccumLim_;

}

Eigen::Vector3f mpccontrol()
{ 
  Eigen::SparseMatrix<double, Eigen::RowMajor> A, B, C, G, H;
  A.resize(6*horizon_N, 6);
  B.resize(6*horizon_N, 3*(horizon_N));
  C.resize(6*horizon_N, 6*horizon_N);
  G.resize(6*horizon_N, 6*horizon_N);
  H.resize(3*horizon_N, 3*horizon_N);

  std::vector<Trip> tripletListA, tripletListB, tripletListC, tripletListG, tripletListH;
  tripletListA.reserve(10*horizon_N);
  tripletListB.reserve((horizon_N*horizon_N+horizon_N)*6);
  tripletListC.reserve(6*horizon_N);
  tripletListG.reserve(6*horizon_N);
  tripletListH.reserve(3*horizon_N);

  //set up matrix A
  for (int i=0; i<horizon_N; i++) {
    for (int j=0; j<6; j++){
      tripletListA.push_back(Trip(i*6+j, j, 1));
    }
    for (int j=0; j<3; j++){
      tripletListA.push_back(Trip(i*6+j, 3+j, T_s*(i+1)));
    }
  }
  A.setFromTriplets(tripletListA.begin(), tripletListA.end());

  //set up matrix B
  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<=i; j++){
      for (int k=0; k<3; k++){
        tripletListB.push_back(Trip(i*6+k, j*3+k, T_s*T_s*(i-j)+T_s*T_s/2));
      }
      for (int k=3; k<6; k++){
        tripletListB.push_back(Trip(i*6+k, j*3+k-3, T_s));
      }
    }
  }
  B.setFromTriplets(tripletListB.begin(), tripletListB.end());

  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<6; j++){
      tripletListC.push_back(Trip(i*6+j, i*6+j, 1));
    }
  }
  C.setFromTriplets(tripletListC.begin(), tripletListC.end());

  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<3; j++){
      tripletListG.push_back(Trip(i*6+j, i*6+j, cost_g_));
      tripletListG.push_back(Trip(i*6+j+3, i*6+j+3, cost_v_));
      tripletListH.push_back(Trip(i*3+j, i*3+j, cost_h_));
    }
  }
  G.setFromTriplets(tripletListG.begin(), tripletListG.end());
  H.setFromTriplets(tripletListH.begin(), tripletListH.end());

  Eigen::SparseMatrix<double, Eigen::RowMajor> _z_r;
  _z_r.resize(6*horizon_N, 1);

  if (!mpc_sim_){
    for (int i=0; i<horizon_N; i++){
      for (int j=0; j<3; j++){
        _z_r.insert(i*6+j, 0) = z_reference_(i*3+j, 0);
        _z_r.insert(i*6+j+3, 0) = 0.0;
      }
        // _z_r.insert(i*6+0, 0) = 4.0;     //for fixed point simulation
        // _z_r.insert(i*6+1, 0) = 0.0;
        // _z_r.insert(i*6+2, 0) = 4.0;   
    }
  }else {
    for (int i=0; i<horizon_N; i++){
      _z_r.insert(i*6+1, 0) = 100.0;
    }
  }

  std::cout<<_z_r<<"\n";
  Eigen::SparseMatrix<double, Eigen::RowMajor> _x_0;
  _x_0.resize(6, 1);
  if (!mpc_sim_){
    _x_0.insert(0, 0) = current_.pos(0);
    _x_0.insert(1, 0) = current_.pos(1);
    _x_0.insert(2, 0) = current_.pos(2);
    _x_0.insert(3, 0) = current_.vel(0);
    _x_0.insert(4, 0) = current_.vel(1);
    _x_0.insert(5, 0) = current_.vel(2);   
  } else {
    _x_0.insert(0, 0) = sim_state_(0);
    _x_0.insert(1, 0) = sim_state_(1);
    _x_0.insert(2, 0) = sim_state_(2);
    _x_0.insert(3, 0) = sim_state_(3);
    _x_0.insert(4, 0) = sim_state_(4);
    _x_0.insert(5, 0) = sim_state_(5);  
  }


   // std::cout<<"big A matrix is"<<A<<"\n";
   // std::cout<<"big B matrix is"<<B<<"\n";
   // std::cout<<"big C matrix is"<<C<<"\n";
   // std::cout<<"big G matrix is"<<G<<"\n";
   // std::cout<<"big H matrix is"<<H<<"\n";

  Eigen::SparseMatrix<double, Eigen::RowMajor> Q = B.transpose()*C.transpose()*G*C*B + H;
  Eigen::MatrixXd bigB1 = _x_0.transpose()*A.transpose()*C.transpose()*G*C*B;
  Eigen::MatrixXd bigB2 = -_z_r.transpose()*G*C*B;
  //Eigen::MatrixXd c = bigB1 + bigB2;
  //Eigen::VectorXd c = Eigen::VectorXd::Zero(3*horizon_N);
  Eigen::VectorXd c = 2* (bigB1 + bigB2).transpose();
  // std::cout<<"big A matrix is"<<Q<<"\n";
  //std::cout<<"big B matrix is"<<c<<"\n";

  Eigen::SparseMatrix<double, Eigen::RowMajor> E;
  E.resize(3*horizon_N, 3*horizon_N);
  for (int i=0; i<3*horizon_N; i++){
    E.insert(i, i) = 1;
    //E.insert(i+3*horizon_N, i) = -1;
  }
  Eigen::VectorXd f(3*horizon_N), g(3*horizon_N);
  for (int i=0; i<horizon_N; i++){
    for (int j=0; j<2; j++){
      f(i*3+j) = -acc_xy_limit_;
      g(i*3+j) = acc_xy_limit_;
    }
      f(i*3+2) = -0.7*ONE_G;
      g(i*3+2) = 1.0*ONE_G;
  }
  //set up ooqp solver
  Eigen::SparseMatrix<double, Eigen::RowMajor> a;
  //a.resize(3*horizon_N, 3*horizon_N);
  Eigen::VectorXd b;//(3*horizon_N);
  Eigen::VectorXd l = Eigen::VectorXd::Constant(3*horizon_N, -10000000);
  Eigen::VectorXd u = Eigen::VectorXd::Constant(3*horizon_N, std::numeric_limits<double>::max());
  Eigen::VectorXd x;
  bool result = ooqpei::OoqpEigenInterface::solve(Q, c, a, b, E, f, g, l, u, x);
  //Eigen::MatrixXd U_k = -bigB*bigA.inverse();
  //std::cout<<"solution is "<<x<<"\n";

  Eigen::Vector3f tarAcc;
  tarAcc(0) = x(0);
  tarAcc(1) = x(1);
  tarAcc(2) = x(2)+ONE_G;

  if (mpc_sim_){
    //tarAcc(2) = tarAcc(2)-ONE_G;
    Eigen::VectorXd tarAccd(3);
    tarAccd(0) =(double)tarAcc(0);
    tarAccd(1) =(double)tarAcc(1);
    tarAccd(2) =(double)(tarAcc(2)-ONE_G);
    Eigen::MatrixXd smallA = Eigen::MatrixXd::Identity(6, 6);
    smallA.topRightCorner(3,3) = Eigen::MatrixXd::Identity(3, 3)*T_s;
    Eigen::MatrixXd smallB(6,3);
    smallB<< T_s*T_s/2, 0, 0, 0, T_s*T_s/2, 0, 0, 0, T_s*T_s/2,
             T_s, 0, 0, 0, T_s, 0, 0, 0, T_s;
    sim_state_ = smallA*sim_state_ + smallB*tarAccd;
    nav_msgs::Odometry odom_sim;
    odom_sim.header.stamp = ros::Time::now();
    odom_sim.pose.pose.position.x = sim_state_(0);
    odom_sim.pose.pose.position.y = sim_state_(1);
    odom_sim.pose.pose.position.z = sim_state_(2);
    mpc_sim_odom.publish(odom_sim);
  }
  std::cout<<"trarget acc x: "<<tarAcc(0)<<"y: "<<tarAcc(1)<<"z: "<<tarAcc(2)<<"\n";
  return tarAcc;
}