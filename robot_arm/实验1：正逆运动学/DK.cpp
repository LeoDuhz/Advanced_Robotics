#include<iostream>
#include<stdio.h>
#include<cmath>
#include <numeric>
#include <vector>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;

#define pi 3.1415926535897932384626

class DK{
    public:
   Eigen::MatrixXd calDK();
   Eigen::MatrixXd calT(double alpha, double a, double d, double w);
   double w[6]{0};

    private:

};

Eigen::MatrixXd DK::calT(double alpha, double a, double d, double w){
    Eigen::MatrixXd T = Eigen::MatrixXd::Ones(4,4);

    T(0,0) = std::cos(w);
    T(0,1) = -std::sin(w);
    T(0,2) = 0;
    T(0,3) = a;
    T(1,0) = std::sin(w)*std::cos(alpha);
    T(1,1) = std::cos(w)*std::cos(alpha);
    T(1,2) = -std::sin(alpha);
    T(1,3) = -std::sin(alpha)*d;
    T(2,0) = std::sin(w)*std::sin(alpha);
    T(2,1) = std::cos(w)*std::sin(alpha);
    T(2,2) = std::cos(alpha);
    T(2,3) = cos(alpha)*d;
    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;

    return T;

}

Eigen::MatrixXd DK::calDK()
{
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);

    T =  T * calT(0,0,0.284,this->w[0]);
    T = T * calT(pi/2,0,0,pi/2+this->w[1]);
    T = T * calT(0,0.225,0,this->w[2]);
    T = T * calT(pi/2,0,0.2289,this->w[3]);
    T = T * calT(-pi/2,0,0,-pi/2+this->w[4]);
    T = T * calT(pi/2,0,0.055,this->w[5]);
    T = T * calT(-pi/2,0,0,0);

    return T;
}
int main(int argc, char **argv)
{
        DK dk;
        for (int i=0;i<6;i++)
        {
            std::cin>>dk.w[i];
        }
 
    Eigen::MatrixXd T = dk.calDK();
    Eigen::MatrixXd xyz = T.block<3,1>(0,3).transpose();
    double pitch = asin(-T(2,0));
    double yaw = asin(T(1,0)/std::cos(pitch));
    double roll = acos(T(2,2)/std::cos(pitch));

    std::cout<<"x,y,z: "<<xyz<<endl;
    std::cout<<"roll: "<<roll<< "       pitch: "<<pitch<<"       yaw: "<<yaw<<endl;

}

