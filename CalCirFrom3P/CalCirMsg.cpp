#include "CalCirMsg.h"
#include <Eigen/Dense>

using namespace Eigen;

CirMsg CalCirFrom3P(Vector3d Ps, Vector3d Pc, Vector3d Pe)
{
	CirMsg re_cirmsg;

	Vector3d r1 = Pc - Ps;
	Vector3d r2 = Pe - Ps;
	r1.normalize();
	r2.normalize();

	if (fabs(r1(0)) == fabs(r2(0)) && fabs(r1(1)) == fabs(r2(1)) && fabs(r1(2)) == fabs(r2(2)))//ÅÐ¶ÏÈýµãÊÇ·ñ¹²Ïß
	{
		re_cirmsg.err_msg = -1;
		return re_cirmsg;
	}

	double x1 = Ps(0);
	double y1 = Ps(1);
	double z1 = Ps(2);
	double x2 = Pc(0);
	double y2 = Pc(1);
	double z2 = Pc(2);
	double x3 = Pe(0);
	double y3 = Pe(1);
	double z3 = Pe(2);

	Matrix3d A;
	A(0, 0) = y2*z1 - y1*z2 + y1*z3 - y3*z1 - y2*z3 + y3*z2;
	A(0, 1) = x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2;
	A(0, 2) = x2*y1 - x1*y2 + x1*y3 - x3*y1 - x2*y3 + x3*y2;
	A(1, 0) = 2 * x2 - 2 * x1;
	A(1, 1) = 2 * y2 - 2 * y1;
	A(1, 2) = 2 * z2 - 2 * z1;
	A(2, 0) = 2 * x3 - 2 * x1;
	A(2, 1) = 2 * y3 - 2 * y1;
	A(2, 2) = 2 * z3 - 2 * z1;

	Vector3d B;
	B(0) = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);
	B(1) = -(x1*x1 - x2*x2 + y1*y1 - y2*y2 + z1*z1 - z2*z2);
	B(2) = -(x1*x1 - x3*x3 + y1*y1 - y3*y3 + z1*z1 - z3*z3);

	Vector3d Po_v = A.inverse()*B;

	re_cirmsg.Po[0] = Po_v(0);
	re_cirmsg.Po[1] = Po_v(1);
	re_cirmsg.Po[2] = Po_v(2);

	Vector3d Ps_v;
	Ps_v << x1, y1, z1;

	Vector3d r_v = Ps_v - Po_v;

	re_cirmsg.r = r_v.norm();
	re_cirmsg.err_msg = 1;

	return re_cirmsg;
}
