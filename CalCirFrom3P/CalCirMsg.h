#ifndef CALCIRMSG_H
#define CALCIRMSG_H

#include <Eigen/Dense>
using namespace Eigen;

//定义存放圆信息结构体
typedef struct CirMsg_t
{
	double Po[3];//圆心
	double r;//半径

	int err_msg;//错误信息，1 正确，-1 三点共线
}CirMsg;

CirMsg CalCirFrom3P(Vector3d Ps, Vector3d Pc, Vector3d Pe);//三点求圆心及半径函数

#endif
