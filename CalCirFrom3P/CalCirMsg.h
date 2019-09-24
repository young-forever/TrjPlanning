#ifndef CALCIRMSG_H
#define CALCIRMSG_H

#include <Eigen/Dense>
using namespace Eigen;

//������Բ��Ϣ�ṹ��
typedef struct CirMsg_t
{
	double Po[3];//Բ��
	double r;//�뾶

	int err_msg;//������Ϣ��1 ��ȷ��-1 ���㹲��
}CirMsg;

CirMsg CalCirFrom3P(Vector3d Ps, Vector3d Pc, Vector3d Pe);//������Բ�ļ��뾶����

#endif
