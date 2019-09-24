#include"CalCirMsg.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define PI 3.14159265354

//�����ֵ�ڵ�<Matrix>Pose_node �� <Vector>t_vec_G
int main()
{
	//�켣�ٶȡ����ٶȲ���
	double v_max = 0.01;
	double a_max = 0.01;
	//�켣����
	VectorXd Ps_6(6);
	Ps_6 << 0.01, 0.0, 0.0, 0, 0, 0;
	VectorXd Pc_6(6);
	Pc_6 << 0.0, 0.01, 0.0, 0, 0, 0;
	VectorXd Pe_6(6);
	Pe_6 << -0.01, 0.0, 0.0, 0, 0, 0;

	//�����ݾ��Ȧ��
	double Ph = 0.02;
	double Cn = 5;


	//��Բ�ļ��뾶
	Vector3d Ps;
	Ps << Ps_6(0), Ps_6(1), Ps_6(2);
	Vector3d Pc;
	Pc << Pc_6(0), Pc_6(1), Pc_6(2);
	Vector3d Pe;
	Pe << Pe_6(0), Pe_6(1), Pe_6(2);

	CirMsg MyCir;
	MyCir = CalCirFrom3P(Ps, Pc, Pe);//�ռ�������Բ�ļ��뾶


	//��ֲ����꼰Բ�淨����
	Map <VectorXd> Po(MyCir.Po, 3);
	Vector3d v = Ps - Po;//�ֲ���������v��
	Vector3d temp_v = Pc - Po;
	Vector3d n = v.cross(temp_v);//Բ�淨������
	Vector3d u = n.cross(v);//�ֲ���������u��
	v.normalize();
	u.normalize();
	n.normalize();

	double r = MyCir.r;

	//��ת�ǲ�����ɢ����
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

	cout << "Բ�ģ�" << MyCir.Po[0] << ", " << MyCir.Po[1] << ", " << MyCir.Po[2] << "." << endl;
	cout << "�뾶�� " << MyCir.r << endl;
	cout << "״̬�� " << MyCir.err_msg << endl << endl;

	cout << "��ֵλ�ã� " << endl;

	for (int i = 0; i <= N; i++)
	{
		cout << Pose_node(0, i) << ", " << Pose_node(1, i) << ", " << Pose_node(2, i) << ", " << Pose_node(3, i) << ", " << Pose_node(4, i) << ", " << Pose_node(5, i) << endl;
	}

	cout << "��ֵʱ�䣺 "<<endl;
	for (int i = 0; i <= N; i++)
	{
		cout << t_vec_G(i) << endl;
	}

	while (1)
		;

	return 0;
}

