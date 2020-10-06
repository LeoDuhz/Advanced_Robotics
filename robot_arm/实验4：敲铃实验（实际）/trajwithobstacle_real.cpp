#include <iostream>
#include <cmath>
#include <string>
#include "vector"
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

#define pi 3.1415926535897932384626

using namespace Eigen;
using namespace std;

VectorXd pathInterpolation3(double init_p, double final_p, double init_v, double final_v, double period);
MatrixXd pathInterp3WithMidpoint(double init_p, double mid_p, double final_p, double whole_period);
VectorXd calcVel(MatrixXd theta_t, double t);

int main(int argc, char** argv)
{
    //节点初始化
    ros::init(argc, argv, "control_example");
    //创建节点句柄对象
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    //创建发布者对象，用于发布位置信息
    ros::Publisher spd_pub = n.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000);
    //设置信息发送频率（50Hz）
    ros::Rate loop_rate(50);

    //初始化publish的变量并初始化6个值
    std_msgs::Float32MultiArray spd_msg;
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);

    //blue ball 0.26 0.15 0.08
    VectorXd blueball_joint_state(6);
    //blueball_joint_state << 0.523405, -1.28345, 0.0903509, -0.00186839, 1.1927, 0.525141; //real
    blueball_joint_state << 0.523405, -1.38345, 0.0903509, -0.00186839, 1.1927, 0.525141; //1.6s version
    //blueball_joint_state << 0.623405, -1.28345, 0.0903509, -0.00186839, 1.1927, 0.525141; 
    // blueball_joint_state << 0.623405, -1.38345, 0.0903509, -0.00186839, 1.1927, 0.525141; 

    //red ball 0.28 -0.24 0.08
    VectorXd redball_joint_state(6);
    // redball_joint_state << -0.708536, -1.44723, 0.565107, -0.000952057, 0.882642, -0.707801;
    //redball_joint_state << -1.208536, -1.64723, 0.565107, -0.000952057, 0.882642, -0.707801; //1.8s version
    redball_joint_state << -1.208536, -1.64723, 0.565107, -0.000952057, 0.882642, -0.707801;//1.6s version
    //redball_joint_state << -1.208536, -1.64723, 0.565107, -0.000952057, 0.882642, -0.707801;
    

    //redball_joint_state << -0.908536, -1.44723, 0.565107, -0.000952057, 0.882642, -0.707801;
   // redball_joint_state << -0.908536, -1.64723, 0.565107, -0.000952057, 0.882642, -0.707801;


/******************************adjust the three parameters to control speed ****************************/
    //middle point, which is supposed to be calculated by inverse kinematics, while replaced with empirical numbers
    VectorXd midpoint_joint_state(6);
    //midpoint_joint_state << 0, -1.2, 0.45, 0, 0.8, 0;  // further from the obstacle,meng's version
    midpoint_joint_state << -0.131790011092997, -1.212079770113469, 0.477135176977792, 0.009279848335710, 0.784490189836066, -0.138183182148361;//1.6s version
//midpoint_joint_state << -0.131790011092997, -0.912079770113469, 0.477135176977792, 0.009279848335710, 0.784490189836066, -0.138183182148361;//1.5s version
    //midpoint_joint_state << 0, -1.4, 0.6, 0, 0.8, 0;  // closer to the obstacle
    //midpoint_joint_state << 0.000115258, -1.15935, 0.450616, -0.00104892, 0.708735, 0.000797974;

    double timeperiod_s2b = 5; // the time spent from start point to reaching the blue ball for the first time
    double timeperiod_r2b = 1.46; // the time spent from red ball to blue ball (same time spent on blue to red)
/******************************adjust the three parameters to control speed****************************/
    
    MatrixXd start2blue_joint_state_interp_param(6,4); // the coeffients of the joint state interpolation polynomial from start to blue ball
    MatrixXd blue2mid_joint_state_interp_param(6,4);
    MatrixXd mid2red_joint_state_interp_param(6,4);
    MatrixXd red2mid_joint_state_interp_param(6,4);
    MatrixXd mid2blue_joint_state_interp_param(6,4);

    // calculate the coefficients of the interpolation polynomials
	for (int i = 0; i < 6; i++){
		start2blue_joint_state_interp_param.block <1, 4>(i, 0) = pathInterpolation3(0, blueball_joint_state(i), 0, 0, timeperiod_s2b).transpose();
        blue2mid_joint_state_interp_param.block <1, 4>(i, 0) = pathInterp3WithMidpoint(blueball_joint_state(i), midpoint_joint_state(i), redball_joint_state(i), timeperiod_r2b).block <1,4>(0,0);
        mid2red_joint_state_interp_param.block <1, 4>(i, 0) = pathInterp3WithMidpoint(blueball_joint_state(i), midpoint_joint_state(i), redball_joint_state(i), timeperiod_r2b).block <1,4>(1,0);
        red2mid_joint_state_interp_param.block <1, 4>(i, 0) = pathInterp3WithMidpoint(redball_joint_state(i), midpoint_joint_state(i), blueball_joint_state(i), timeperiod_r2b).block <1,4>(0,0);
        mid2blue_joint_state_interp_param.block <1, 4>(i, 0) = pathInterp3WithMidpoint(redball_joint_state(i), midpoint_joint_state(i), blueball_joint_state(i), timeperiod_r2b).block <1,4>(1,0);
	}

    //from start point to reaching blue ball for the first time
    double t = 0.0;
    const double timestep = 0.02;
    VectorXd vel(6);
    while (ros::ok())
    {
        if (t >= timeperiod_s2b)  break;

        vel = calcVel(start2blue_joint_state_interp_param, t);

        spd_msg.data.at(0) = vel(0) * 30 * 180 / pi;
        spd_msg.data.at(1) = vel(1) * 205 * 180 / (3 * pi);
        spd_msg.data.at(2) = vel(2) * 50 * 180 / pi;
        spd_msg.data.at(3) = vel(3) * 125 * 180 / (2 * pi);
        spd_msg.data.at(4) = vel(4) * 125 * 180 / (2 * pi);
        spd_msg.data.at(5) = vel(5) * 200 * 180 / (9 * pi);
        spd_pub.publish(spd_msg);
        ROS_INFO_STREAM("published");

        t = t + timestep;
        loop_rate.sleep();
    }
   double max_vel = 0;
    //back and forth between blue and red ball
    int current_state = 0; // 0: blue2mid, 1: mid2red, 2: red2mid, 3: mid2blue
    while (ros::ok()){
        for(t = 0; t <= timeperiod_r2b / 2; t += timestep, loop_rate.sleep()){
            switch(current_state){
                case 0: 
                    vel  = calcVel (blue2mid_joint_state_interp_param, t);
                    break;
                case 1:
                    vel  = calcVel (mid2red_joint_state_interp_param, t);
                    break;
                case 2:
                    vel  = calcVel (red2mid_joint_state_interp_param, t);
                    break;
                case 3:
                    vel = calcVel (mid2blue_joint_state_interp_param, t);
                    break;
            }

            spd_msg.data.at(0) = vel(0) * 30 * 180 / pi;
            spd_msg.data.at(1) = vel(1) * 205 * 180 / (3 * pi);
            spd_msg.data.at(2) = vel(2) * 50 * 180 / pi;
            spd_msg.data.at(3) = vel(3) * 125 * 180 / (2 * pi);
            spd_msg.data.at(4) = vel(4) * 125 * 180 / (2 * pi);
            spd_msg.data.at(5) = vel(5) * 200 * 180 / (9 * pi);
            spd_pub.publish(spd_msg);
            ROS_INFO_STREAM("published");
        }
        current_state += 1;
        if(current_state == 4) current_state = 0;
    }
    return 0;
}


//Interpolation: p = a0 + a1 * t + a2 * t^2 + a3 * t^3
//arg: init_p, final_p, init_v, final_v, period
//return: [a0, a1, a2, a3]
VectorXd pathInterpolation3(double init_p, double final_p, double init_v, double final_v, double period){
	VectorXd param(4);

	param(0) = init_p;
	param(1) = init_v;
	param(2) =  3 * (final_p - init_p) / pow(period, 2) - 2 * init_v / period - final_v / period;
	param(3) = -2 * (final_p - init_p) / pow(period, 3) + (final_v + init_v) / pow(period, 2);

	return param;
}

//Interpolation with midpoint: p = a0 + a1 * t + a2 * t^2 + a3 * t^3
//arg: init_p, final_p, init_v, final_v, whole_period
//return: [a10, a11, a12, a13
//                 a20, a21, a22, a23]
MatrixXd pathInterp3WithMidpoint(double init_p, double mid_p, double final_p, double whole_period){
    //tf1 = tf2
    double tf = whole_period / 2;
    MatrixXd A(2, 4);

    A(0, 0) = init_p;
    A(0, 1) = 0;
    A(0, 2) = (12 * mid_p - 3 * final_p - 9 * init_p) / (4 * tf * tf);
    A(0, 3) = (-8 * mid_p + 3 * final_p + 5 * init_p) / (4 * tf * tf * tf);
    A(1, 0) = mid_p;
    A(1, 1) = (3 * final_p - 3 * init_p) / (4 * tf);
    A(1, 2) = (-12 * mid_p + 6 * final_p + 6 * init_p) / (4 * tf * tf);
    A(1, 3) = (8 * mid_p - 5 * final_p - 3 * init_p) / (4 * tf * tf * tf);

    return A;
}

VectorXd calcVel(MatrixXd theta_t, double t){
    VectorXd v(6);  
    for (int i = 0; i < 6; i++) {
        v(i) = theta_t(i, 1) + 2 * theta_t(i, 2) * t + 3 * theta_t(i, 3) * t * t;
    }
    return v;
}