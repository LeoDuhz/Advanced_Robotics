#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#define pi 3.1415926535897932384626
using namespace std;
class Anno_robot{
public:
	Anno_robot(Eigen::MatrixXd DHparam);
	~Anno_robot() {};
	Eigen::VectorXd getJointState();
	void setEndVelocity(Eigen::VectorXd);
	Eigen::MatrixXd getJacobian();
	Eigen::MatrixXd calcTransform(double a, double b, double d, double o);
	void printParam();
	static double timestep;
	Eigen::VectorXd joint_velocity;

private:
	Eigen::MatrixXd DHparam;
	Eigen::VectorXd end_velocity;
	
};

double Anno_robot::timestep = 0.02;

Anno_robot::Anno_robot(Eigen::MatrixXd DHparam) {
	this->DHparam = DHparam;
	end_velocity = Eigen::VectorXd::Zero(6,1);
	joint_velocity = Eigen::VectorXd::Zero(6,1);
}

Eigen::VectorXd Anno_robot::getJointState() {
	return DHparam.block<6, 1>(0, 3);
}

void Anno_robot::setEndVelocity(Eigen::VectorXd V) {
	this->end_velocity = V;
	Eigen::MatrixXd J = this->getJacobian();
	
	if (J.determinant() != 0) {
		this->joint_velocity = J.inverse() * V;
	}
	else {
		std::cout << "�ſɱȾ������죡�޷������ٶ�" << std::endl;
		//try {
			//throw 0;
		//}
		Eigen::VectorXd delta;
	}

	Eigen::VectorXd delta_joint_state = Anno_robot::timestep * this->joint_velocity;
	DHparam.block<6, 1>(0, 3) += delta_joint_state;

}

Eigen::MatrixXd Anno_robot::getJacobian(){
	Eigen::MatrixXd P(3, 6);
	Eigen::MatrixXd Z(3, 6);
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);

	for (int i = 0; i < 6; i++) {
		T = T * calcTransform(DHparam(i,0), DHparam(i,1), DHparam(i,2), DHparam(i,3));
		P.block<3, 1>(0, i) = T.block<3, 1>(0, 3);
		Z.block<3, 1>(0, i) = T.block<3, 1>(0, 2);

	}

	Eigen::MatrixXd J(6, 6);
	Eigen::Vector3d z(3);
	Eigen::Vector3d p(3);
	Eigen::Vector3d pN = P.block<3, 1>(0, 5);

	for (int i = 0; i < 5; i++){
		z = Z.block<3, 1>(0, i);
		p = P.block<3, 1>(0, i);
		J.block<3, 1>(0, i) = z.cross(pN - p);
		J.block<3, 1>(3, i) = z;
	}
	J.block<3, 1>(0, 5) = Eigen::VectorXd::Zero(3, 1);
	J.block<3, 1>(3, 5) = Z.block<3, 1>(0, 5);

	return J;
}

Eigen::MatrixXd Anno_robot::calcTransform(double b, double a, double d, double o)
{
	Eigen::MatrixXd T(4, 4);   // T[i-1 -> i]
	T << cos(o), -sin(o), 0, a,
		sin(o)* cos(b), cos(o)* cos(b), -sin(b), -sin(b) * d,
		sin(o)* sin(b), cos(o)* sin(b), cos(b), cos(b)* d,
		0, 0, 0, 1;
	return T;
}

void Anno_robot::printParam()
{
	
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);

	for (int i = 0; i < 6; i++) {
		T = T * calcTransform(DHparam(i, 0), DHparam(i, 1), DHparam(i, 2), DHparam(i, 3));
	}

	Eigen::MatrixXd Final(4, 4);
	Final << 1, 0, 0, 0,
		0, 0, 1, 0,
		0, -1, 0, 0,
		0, 0, 0, 1;
	T = T * Final;

	std::cout << "transformation matrix T:" << std::endl;
	std::cout << T << std::endl;

	double yaw, pitch, roll;
	pitch = -asin(T(2, 0));
	yaw = asin(T(1, 0) / cos(pitch));
	roll = asin(T(2, 1) / cos(pitch));

	std::cout << "x = " << T(0, 3) << "  " << "y = " << T(1, 3) << "  " << "z = " << T(2, 3) << std::endl;
	std::cout << "Roll = " << roll << "  " << "Pitch = " << pitch << "  " << "Yaw = " << yaw << "  " << std::endl;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_robot_vel");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    ros::Publisher spd_pub = n.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
    std_msgs::Float64MultiArray spd_msg;
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    spd_msg.data.push_back(0.0);
    ros::Rate loop_rate(50);
	Eigen::MatrixXd myDHparam(6, 4);

	myDHparam <<	0,			0,		0.284,	0,
					pi / 2,		0,		0,		pi / 2,
					0,			0.225,	0,		0 ,
					pi / 2,		0,		0.2289, 0 ,
					-pi / 2,	0,		0,		-pi / 2 ,
					pi / 2,		0,		0.055,	0;
	Anno_robot myrobot(myDHparam);
	double time = 0.02;
	Eigen::VectorXd V = Eigen::VectorXd::Zero(6,1);

	while (time <= 12) {
		if (time >= 0 && time <= 4) {
			V(0) = 0.003535 * time;
		}
		else if (time > 4 && time <= 7) {
			V(0) = 0.01414;
		}
		else if (time > 7 && time <= 12) {
			V(0) = 0.033936 - 0.002828 * time;
		}
		V(2) = -V(0);
		
		//try {
		myrobot.setEndVelocity(V);

        std::cout<<"joint velocity:"<<myrobot.joint_velocity<<endl;
        spd_msg.data.at(0) = myrobot.joint_velocity(0);
        spd_msg.data.at(1) = myrobot.joint_velocity(1);
        spd_msg.data.at(2) = myrobot.joint_velocity(2);
        spd_msg.data.at(3) = myrobot.joint_velocity(3);
        spd_msg.data.at(4) = myrobot.joint_velocity(4);
        spd_msg.data.at(5) = myrobot.joint_velocity(5);

         spd_pub.publish(spd_msg);
         loop_rate.sleep();
        //  sleep(0.02);
		//}
		//catch (int a) {
		//	if (a == 0) std::cout << "failed to set end velocity at sigular point!" << std::endl;
		//}
		// std::cout << "End Velocity = " << V << std::endl;
		// std::cout << "Joint State = " << myrobot.getJointState() << std::endl;

		time += Anno_robot::timestep;
	}
	
	std::cout << "Final Joint State = " << myrobot.getJointState() << std::endl;
	myrobot.printParam();
}

