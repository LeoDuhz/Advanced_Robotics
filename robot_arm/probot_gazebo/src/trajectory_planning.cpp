#include <iostream>
#include <string>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

#define pi 3.1415926535897932384626

using namespace std;

Eigen::VectorXd pathInterpolation3(double init_p, double final_p, double init_v, double final_v, double period);
Eigen::VectorXd calcVel(Eigen::MatrixXd theta_t, double t);

int main(int argc, char** argv) {
	// End Joint State
	Eigen::VectorXd end_joint_state(6);
	end_joint_state << -0.708536, -1.44723, 0.565107, -0.000952057, 0.882642, -0.707801;

	// for each joint state theta, theta = a0 + a1 * t + a2 * t^2 + a3 * t^3
	// joint_state_interp_param stores the interpolation parameters a0, a1, a2, a3
	// one row for one joint_state theta
	Eigen::MatrixXd joint_state_interp_param(6,4);
	for (int i = 0; i < 6; i++)
	{
		joint_state_interp_param.block <1, 4>(i, 0) = pathInterpolation3(0, end_joint_state(i), 0, 0, 10).transpose();
	}

	//节点初始化
    ros::init(argc, argv, "traj");
    //创建节点句柄对象
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    //创建发布者对象，用于发布位置信息
    ros::Publisher spd_pub = n.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 1000);
    //设置信息发送频率（50Hz）
    ros::Rate loop_rate(50);

    //初始化publish的变量并初始化6个值
    std_msgs::Float64MultiArray spd_msg;
    spd_msg.data.push_back(0);
    spd_msg.data.push_back(0);
    spd_msg.data.push_back(0);
    spd_msg.data.push_back(0);
    spd_msg.data.push_back(0);
    spd_msg.data.push_back(0);

	double t = 0.0;
	Eigen::VectorXd v(6);
    while (ros::ok())
    {
        v  = calcVel (joint_state_interp_param, t);
        if(t >= 10) break;
        //为要发送的变量装入解出的六关节坐标
        spd_msg.data.at(0) = v(0);
        spd_msg.data.at(1) = v(1);
        spd_msg.data.at(2) = v(2);
        spd_msg.data.at(3) = v(3);
        spd_msg.data.at(4) = v(4);
        spd_msg.data.at(5) = v(5);

        //发送出去，若成功，机械臂状态会改变
        spd_pub.publish(spd_msg);
        cout << v << endl;

        ROS_INFO_STREAM("published");

        t = t + 0.02;
        loop_rate.sleep();
        cout << "t: " << t << endl;
    }	

	return 0;
}

//Interpolation: p = a0 + a1 * t + a2 * t^2 + a3 * t^3
//arg: init_p, final_p, init_v, final_v, period
//return: [a0, a1, a2, a3]
Eigen::VectorXd pathInterpolation3(double init_p, double final_p, double init_v, double final_v, double period)
{
	Eigen::VectorXd param(4);

	param(0) = init_p;
	param(1) = init_v;
	param(2) =  3 * (final_p - init_p) / pow(period, 2) - 2 * init_v / period - final_v / period;
	param(3) = -2 * (final_p - init_p) / pow(period, 3) + (final_v + init_v) / pow(period, 2);

	return param;
}

Eigen::VectorXd calcVel(Eigen::MatrixXd theta_t, double t){
    Eigen::VectorXd v(6);  
    for (int i = 0; i < 6; i++) {
        v(i) = theta_t(i, 1) + 2 * theta_t(i, 2) * t + 3 * theta_t(i, 3) * t * t;
    }
    return v;
}