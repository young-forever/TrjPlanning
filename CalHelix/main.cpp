#include"CalCirMsg.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI 3.14159265354

//输出插值节点<Matrix>Pose_node 和 <Vector>t_vec_G
int main()
{
	//轨迹速度、加速度参数
	double v_max = 0.01;
	double a_max = 0.01;
	//轨迹参数
	VectorXd Ps_6(6);
	Ps_6 << 0.01, 0.0, 0.0, 0, 0, 0;
	VectorXd Pc_6(6);
	Pc_6 << 0.0, 0.01, 0.0, 0, 0, 0;
	VectorXd Pe_6(6);
	Pe_6 << -0.01, 0.0, 0.0, 0, 0, 0;

	//定义螺距和圈数
	double Ph = 0.02;
	double Cn = 5;


	//求圆心及半径
	Vector3d Ps;
	Ps << Ps_6(0), Ps_6(1), Ps_6(2);
	Vector3d Pc;
	Pc << Pc_6(0), Pc_6(1), Pc_6(2);
	Vector3d Pe;
	Pe << Pe_6(0), Pe_6(1), Pe_6(2);

	CirMsg MyCir;
	MyCir = CalCirFrom3P(Ps, Pc, Pe);//空间三点求圆心及半径


	//求局部坐标及圆面法向量
	Map <VectorXd> Po(MyCir.Po, 3);
	Vector3d v = Ps - Po;//局部正交向量v；
	Vector3d temp_v = Pc - Po;
	Vector3d n = v.cross(temp_v);//圆面法向量；
	Vector3d u = n.cross(v);//局部正交向量u；
	v.normalize();
	u.normalize();
	n.normalize();

	double r = MyCir.r;

	//得转角参数离散向量
	int N = (int)ceil(Cn * 20);
	VectorXd theta_v(N+1);
	for (int i = 0; i <= N; i++)
	{
		theta_v(i) = i * 2 * PI / 20;
	}

	MatrixXd Pose_node(6, N+1);
	Vector3d P_temp;
	for (int i = 0; i <= N; i++)
	{
		P_temp = Po + r*(v*cos(theta_v(i)) + u*sin(theta_v(i))) + theta_v(i)/2/PI*Ph*n;
		Pose_node(0, i) = P_temp(0);
		Pose_node(1, i) = P_temp(1);
		Pose_node(2, i) = P_temp(2);
		Pose_node(3, i) = Ps_6(3);
		Pose_node(4, i) = Ps_6(4);
		Pose_node(5, i) = Ps_6(5);
	}

	double w_max = v_max / r;
	double al_max = a_max / r;
	double t_a = w_max / al_max;    //time of acc to v_max;
	double len_a = 0.5*al_max*pow(t_a, 2);//length of acc to v_max.

	double length = 2 * PI*Cn;

	VectorXd t_vec_G(N+1);
	if (len_a > 0.5*length)
	{
		t_a = pow(length / al_max, 0.5);
		for (int i_cal = 0; i_cal <= N; i_cal++)
		{
			if (theta_v(i_cal) <= length / 2)
			{
				t_vec_G(i_cal) = pow(2 * theta_v(i_cal) / al_max, 0.5);
			}
			else
			{
				t_vec_G(i_cal) = 2 * t_a - (pow(2 * (length - theta_v(i_cal)) / al_max, 0.5));
			}
		}
	}
	else
	{
		for (int i_cal = 0; i_cal <= N; i_cal++)
		{
			if (theta_v(i_cal) <= len_a)
			{
				t_vec_G(i_cal) = pow(2 * theta_v(i_cal) / al_max, 0.5);
			}

			else if (theta_v(i_cal)>len_a && theta_v(i_cal)<length - len_a)
			{
				t_vec_G(i_cal) = pow(2 * len_a / al_max, 0.5) + (theta_v(i_cal) - len_a) / w_max;
			}
			else
			{
				t_vec_G(i_cal) = 2 * pow((2 * len_a / al_max), 0.5) + (length - 2 * len_a) / w_max - pow(2 * (length - theta_v(i_cal)) / al_max, 0.5);
			}
		}
	}

	cout << "圆心：" << MyCir.Po[0] << ", " << MyCir.Po[1] << ", " << MyCir.Po[2] << "." << endl;
	cout << "半径： " << MyCir.r << endl;
	cout << "状态： " << MyCir.err_msg << endl << endl;

	cout << "插值位置： " << endl;

	for (int i = 0; i <= N; i++)
	{
		cout << Pose_node(0, i) << ", " << Pose_node(1, i) << ", " << Pose_node(2, i) << ", " << Pose_node(3, i) << ", " << Pose_node(4, i) << ", " << Pose_node(5, i) << endl;
	}

	cout << "插值时间： "<<endl;
	for (int i = 0; i <= N; i++)
	{
		cout << t_vec_G(i) << endl;
	}

	while (1)
		;

	return 0;
}

