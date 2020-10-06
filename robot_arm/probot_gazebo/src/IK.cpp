#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
// #define pi 3.145926535897932384626

double pi = 2*acos(0);

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

class IK{
public:
    double pose[6]{0};//x,y,z,c,b,a
    double x = 0, y = 0, z = 0, c = 0, b = 0, a = 0;
    Eigen::Matrix4d T06;
    Eigen::MatrixXd Tt;
    double d6 = 0.055, d1 = 0.284, a2 = 0.225, d4 = 0.2289;
    void calT06();
    Eigen::MatrixXd cal_theta();
    int check_angular(double w, double a, double b);
 };

void IK::calT06()
{
    Eigen::MatrixXd T06(4,4);
    T06 << std::cos(a)*std::cos(b), std::cos(a)*std::sin(b)*std::sin(c)-std::sin(a)*std::cos(c), std::cos(a)*std::sin(b)*std::cos(c)+std::sin(a)*std::sin(c),  x,
    std::sin(a)*std::cos(b), std::sin(a)*std::sin(b)*std::sin(c)+std::cos(a)*std::cos(c), std::sin(a)*std::sin(b)*std::cos(c)-std::cos(a)*std::sin(c), y,
    -std::sin(b), std::cos(b)*std::sin(c), std::cos(b)*std::cos(c), z,
    0,0,0,1;

    this->Tt = T06;
    Eigen::MatrixXd trans(4,4);
    trans << 1,0,0,0,
    0,0,-1,0,
    0,1,0,0,
    0,0,0,1;

    this->T06 = T06 * trans;
    return;
}

Eigen::MatrixXd IK::cal_theta()
{
    //calculate theta 1
    double theta1[2]{0};
    theta1[0] = atan((d6*T06(1,2)-y) / (d6*T06(0,2)-x));
    theta1[1] = theta1[0] + pi;

    //calculate theta 3
    double theta3[4]{0};
    theta3[0] = asin((pow((std::cos(theta1[0])*x-d6*std::cos(theta1[0])*T06(0,2)-T06(1,2)*std::sin(theta1[0])*d6+std::sin(theta1[0])*y),2)+pow((z-d1-d6*T06(2,2)),2)-pow(a2,2)-pow(d4,2))/(2*a2*d4));
    theta3[1] = pi - asin((pow((std::cos(theta1[0])*x-d6*std::cos(theta1[0])*T06(0,2)-T06(1,2)*std::sin(theta1[0])*d6+std::sin(theta1[0])*y),2)+pow((z-d1-d6*T06(2,2)),2)-pow(a2,2)-pow(d4,2))/(2*a2*d4));
    theta3[2] = asin((pow((std::cos(theta1[1])*x-d6*std::cos(theta1[1])*T06(0,2)-T06(1,2)*std::sin(theta1[1])*d6+std::sin(theta1[1])*y),2)+pow((z-d1-d6*T06(2,2)),2)-pow(a2,2)-pow(d4,2))/(2*a2*d4));
    theta3[3] = pi -  asin((pow((std::cos(theta1[1])*x-d6*std::cos(theta1[1])*T06(0,2)-T06(1,2)*std::sin(theta1[1])*d6+std::sin(theta1[1])*y),2)+pow((z-d1-d6*T06(2,2)),2)-pow(a2,2)-pow(d4,2))/(2*a2*d4));

    //calculate theta2
    double theta2[8]{0};
    int t = 0;
    for(int i=0;i<8;i++)
    {
        t = i % 4;
        
        double temp1 = sqrt(pow((d4*std::sin(theta3[t])+a2),2)+d4*d4*std::cos(theta3[t])*cos(theta3[t]));
        double temp2 = acos((d4*sin(theta3[t])+a2)/temp1);

        if(i % 2 == 0)
        {
            theta2[i] = asin((z-d6*T06(2,2)-d1)/temp1)+temp2;
        }
        else
        {
            theta2[i] = pi - (asin((z-d6*T06(2,2)-d1)/temp1)+temp2);
        }

    }

    //calculate theta5
    double theta5[16]{0};
    for(int i=0;i<16;i++)
    {
        int t1 = i / 8;
        int t3 = i / 4;
        int t2 = i / 2;
        if(i % 2 == 0)
        {
            theta5[i] = acos(T06(1,2)*std::sin(theta1[t1])*std::sin(theta2[t2]+theta3[t3])+std::cos(theta1[t1])*T06(0,2)*std::sin(theta2[t2]+theta3[t3])-T06(2,2)*std::cos(theta2[t2]+theta3[t3]));
        }
        else{
            theta5[i] = -acos(T06(1,2)*std::sin(theta1[t1])*std::sin(theta2[t2]+theta3[t3])+std::cos(theta1[t1])*T06(0,2)*std::sin(theta2[t2]+theta3[t3])-T06(2,2)*std::cos(theta2[t2]+theta3[t3]));
        }
    }

    //calculate theta4 & theta6
    double theta4[32]{0};
    double theta6[32]{0};
    Eigen::MatrixXd theta(32,6);
    // double theta[32][6]{0};
    for(int i=0;i<32;i++)
    {
        int t1 = i / 16;
        int t2 = i / 8;
        int t3 = i /4;
        int t4 = i / 2;

        if(i % 2 == 0)
        {
            theta4[i] = asin((-std::cos(theta1[t1])*T06(1,2)+T06(0,2)*std::sin(theta1[t1]))/std::sin(theta5[t4]));
            double temp1 = T06(0,1)*std::cos(theta1[t1])*std::sin(theta2[t3]+theta3[t2])+T06(1,1)*std::sin(theta1[t1])*std::sin(theta2[t3]+theta3[t2])-T06(2,1)*std::cos(theta2[t3]+theta3[t2]);
            theta6[i] = asin(temp1/std::sin(theta5[t4]));
        }
        else{
            theta4[i] = pi - asin((-std::cos(theta1[t1])*T06(1,2)+T06(0,2)*std::sin(theta1[t1]))/std::sin(theta5[t4]));
            double temp1 = T06(0,1)*std::cos(theta1[t1])*std::sin(theta2[t3]+theta3[t2])+T06(1,1)*std::sin(theta1[t1])*std::sin(theta2[t3]+theta3[t2])-T06(2,1)*std::cos(theta2[t3]+theta3[t2]);
            theta6[i] = pi - asin(temp1/std::sin(theta5[t4]));
        }

        Eigen::MatrixXd temp(1,6);
        temp << theta1[t1], theta2[t3]-pi/2,theta3[t2], theta4[i], theta5[t4]+pi/2, theta6[i];

        theta.block<1,6>(i,0) = temp;
    }

    Eigen::MatrixXd result(32,6);
    int count = -1;
    for(int i=0;i<32;i++)
    {
        if (check_angular(theta(i,0), -pi, pi) == -1) 
        {
            continue;
        }
        else{
            theta(i,0) = theta(i,0) + check_angular(theta(i,0), -pi, pi) * pi;
        }
        if (check_angular(theta(i,1), -2.01, 2.01) == -1)
        {
            continue;
        }
        else{
            theta(i,1) = theta(i,1) + check_angular(theta(i,1), -2.01, 2.01) * pi;
        }
        if (check_angular(theta(i,2), -0.69, 3.83) == -1)
        {
            continue;
        }
        else{
            theta(i,2) = theta(i,2) + check_angular(theta(i,2), -0.69, 3.83) * pi;
        }
        if (check_angular(theta(i,3), -pi, pi) == -1) 
        {
            continue;
        }
        else{
            theta(i,3) = theta(i,3) + check_angular(theta(i,3), -pi, pi) * pi;
        }
        if (check_angular(theta(i,4), -0.78, 3.92) == -1)
        {
            continue;
        }
        else{
            theta(i,4) = theta(i,4) + check_angular(theta(i,4), -0.78, 3.92) * pi;
        }
        if (check_angular(theta(i,5), -pi, pi) == -1)
        {
            continue;
        }
        else{
            theta(i,5) = theta(i,5) + check_angular(theta(i,5), -pi, pi) * pi;
        }

        count += 1;
        Eigen::MatrixXd tmp(1,6);
        tmp << theta(i,0), theta(i,1), theta(i,2), theta(i,3), theta(i,4), theta(i,5);
        result.block<1,6>(count,0) =  tmp;

    }

    DK dk;
    Eigen::MatrixXd final_result(32,6);
    int cnt = 0;
    for(int i=0;i<=count;i++)
    {
        for(int j=0;j<6;j++)
        {
            dk.w[j] = result(i,j);
        }
        Eigen::MatrixXd T = dk.calDK();
        Eigen::MatrixXd dT = T - Tt;
        double sum = 0;
        for(int k=0;k<4;k++)
        {
            for(int m=0;m<4;m++)
            {
                sum += abs(dT(k,m));
            }
        }
        if (sum < 0.0001)
        {
            Eigen::MatrixXd tmp(1,6);
            tmp << result(i,0), result(i,1), result(i,2), result(i,3), result(i,4), result(i,5);
            final_result.block<1,6>(cnt,0) = tmp;
            cnt += 1;
            // std::cout<<cnt<<endl;
        }

    }

    Eigen::MatrixXd final_matrix = final_result.block(0, 0, cnt, 6);


    // Eigen::MatrixXd final_result(count+1,6);
    // final_result = result.block<count+1,6>(0,0);

    return final_matrix;
}

int IK::check_angular(double w, double a, double b)
{
    int flag = -1;
    if ((a < w) && (w < b))
    {
        flag = 0;
    }

    if ((a < w + 2 * pi) && (w + 2 * pi < b))
    {
        flag = 2;
    }

    if ((a < w - 2 * pi) && (w - 2 * pi < b))
    {
        flag = -2;
    }

    return flag;

}

int main(int argc, char **argv)
{
        std::cout<<"please input x,y,z,roll,pitch,yaw:"<<endl;
        IK ik;
        std::cin>>ik.x>>ik.y>>ik.z>>ik.c>>ik.b>>ik.a;
        ik.calT06();
        Eigen::MatrixXd theta;
        theta = ik.cal_theta();
        std::cout<<"Theta:"<<endl<<theta<<endl;
}
 