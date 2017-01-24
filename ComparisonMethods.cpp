/*
 * ComparisonMethods.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: atabb
 */


#include "ComparisonMethods.hpp"
//#include "levmar.h"


void Q_computation(ColumnVector& quat, Matrix& Q){

	double q0 = quat(1);
	double qx = quat(2);
	double qy = quat(3);
	double qz = quat(4);

	Q.Row(1) << q0 << -qx << -qy << -qz;
	Q.Row(2) << qx << q0 << -qz << qy;
	Q.Row(3) << qy << qz << q0 << -qx;
	Q.Row(4) << qz << -qy << qx << q0;

}

void W_computation(ColumnVector& quat, Matrix& W){

	double q0 = quat(1);
	double qx = quat(2);
	double qy = quat(3);
	double qz = quat(4);

	W.Row(1) << q0 << -qx << -qy << -qz;
	W.Row(2) << qx << q0 << qz << -qy;
	W.Row(3) << qy << -qz << q0 << qx;
	W.Row(4) << qz << qy << -qx << q0;

}

void ConvertQuaternionToMatrix(ColumnVector& q, Matrix& Rx){
	//	Matrix Rx(3, 3);


	double w, x, y, z;
	w = q(1);
	x = q(2);
	y = q(3);
	z = q(4);

	Rx(1, 1)= 1  - 2.0*y*y - 2.0*z*z;
	Rx(2, 2)= 1 - 2.0*x*x - 2.0*z*z;
	Rx(3, 3)= 1 - 2.0*x*x - 2.0*y*y;

	Rx(1, 2)= 2.0*(x*y - z*w);
	Rx(2, 1)= 2.0*(x*y + z*w);

	Rx(1, 3)= 2.0*(x*z + y*w);
	Rx(3, 1)= 2.0*(x*z - y*w);

	Rx(2, 3)= 2.0*(y*z - x*w);
	Rx(3, 2)= 2.0*(y*z + x*w);

	//	double w, x, y, z;
	//	w = q(1);
	//	x = q(2);
	//	y = q(3);
	//	z = q(4);
	//
	//	Rx(1, 1) = 1 - 2.0*y*y - 2.0*z*z;
}

void ConvertRotationMatrixToQuaternion(Matrix& R, ColumnVector& n){

	// http://www.slideshare.net/krizie07/quaternion-to-matrix-matrix-to-quaternion
	// Note that this function is unstable if R is an identity matrix
	// How to check: compute r = sqrt(1 + first diagonal element - (the sum of the other two diagonal elements))
	// If r is zero or very small, then the quaterion should be the identity quaternion (1, 0, 0, 0)
	double r;
	r = sqrt(1 + R(1,1) - R(2,2) - R(3,3));

	if (r==0)
	{
		n=0;
		n(1) =1;
	}
	else
	{
		double angle = acos(0.5*( R(1, 1) +  R(2, 2) + R(3, 3) - 1));
		n(1) = angle;

		n(2) = (R(3, 2) - R(2, 3))/(2.0*sin(angle));
		n(3) = (R(1, 3) - R(3, 1))/(2.0*sin(angle));
		n(4) = (R(2, 1) - R(1, 2))/(2.0*sin(angle));

		n(1) = cos(0.5*angle);
		for (int i = 2; i <= 4; i++){
			n(i) = n(i)*sin(0.5*angle);
		}


		if (n.SubMatrix(2, 4, 1, 1).NormFrobenius() > 1.01){
			cout << "Not unit " << n.t() << endl;

			cout << " R " << endl;
			cout << R << endl << " R.t()*R " << endl << R.t()*R << endl;
		}

		//cout << "length " << n.NormFrobenius() << endl;

		n = n/n.NormFrobenius();
	}

}


void ConvertFromMatrixToDouble(double* D, Matrix& M, int rows, int cols){

	// matrix will hold the sizing information ....
	for (int i = 0, index = 0; i < rows; i++){
		for (int j=  0; j < cols; j++, index++){
			D[index] = M(i + 1, j + 1);
		}
	}

}


void ConvertFromDoubleToMatrix(double* D, Matrix& M, int rows, int cols){

	// matrix will hold the sizing information ....
	for (int i = 0, index = 0; i < rows; i++){
		for (int j=  0; j < cols; j++, index++){
			M(i + 1, j + 1) = D[index];
		}
	}

}


void Convert3x4MatrixToDualQuaternion(Matrix& A, ColumnVector& dual_quat){

	// http://www.slideshare.net/krizie07/quaternion-to-matrix-matrix-to-quaternion
	// http://www.xbdev.net/misc_demos/demos/dual_quaternions_beyond/paper.pdf
	// Note that this function is unstable if R is an identity matrix
	// How to check: compute r = sqrt(1 + first diagonal element - (the sum of the other two diagonal elements))
	// If r is zero or very small, then the dula quaterion should be the identity dula quaternion (1, 0, 0, 0)(0, 0, 0, 0)
	double r;
	r = sqrt(1 + A(1,1) - A(2,2) - A(3,3));

	if (r==0)
	{
		dual_quat=0;
		dual_quat(1) =1;
	}
	else
	{

		// code for this function follows math.cpp from anima-animation-sample
		//https://code.google.com/p/anima-animation-sample/source/browse/trunk/math.cpp
		ColumnVector quaternion(4);
		ColumnVector dual(4);
		ConvertRotationMatrixToQuaternion(A, quaternion);

		const float qx = quaternion(2), qy = quaternion(3), qz = quaternion(4), qw = quaternion(1);
		const float tx = A(1, 4), ty = A(2, 4), tz = A(3, 4);

		// scalar part first, then vector part...
		dual << -0.5*( tx*qx + ty*qy + tz*qz) << 0.5*( tx*qw + ty*qz - tz*qy) <<  0.5*(-tx*qz + ty*qw + tz*qx)
																																							 <<  0.5*( tx*qy - ty*qx + tz*qw);

		dual_quat.SubMatrix(1, 4, 1, 1) = quaternion;
		dual_quat.SubMatrix(5, 8, 1, 1) = dual;

		// normalize

		float realNorm = quaternion.NormFrobenius();
		if( realNorm > 0 )
		{
			quaternion = quaternion/realNorm;

			dual = dual/realNorm;

			dual_quat.SubMatrix(1, 4, 1, 1) = quaternion;
			// this makes the dual part have a norm of 0
			dual_quat.SubMatrix(5, 8, 1, 1) = dual - DotProduct(quaternion, dual) * quaternion;
		}
	}
}




void NormalizeDualQuaternion(ColumnVector& dual_quat){

	ColumnVector quaternion(4);
	ColumnVector dual(4);

	quaternion = dual_quat.SubMatrix(1, 4, 1, 1);
	dual= dual_quat.SubMatrix(5, 8, 1, 1);

	float realNorm = quaternion.NormFrobenius();
	if( realNorm > 0 )
	{
		quaternion = quaternion/realNorm;

		dual = dual/realNorm;

		dual_quat.SubMatrix(1, 4, 1, 1) = quaternion;
		// this makes the dual part have a norm of 0
		dual_quat.SubMatrix(5, 8, 1, 1) = dual - DotProduct(quaternion, dual) * quaternion;
	}

}
void ConvertDualQuaternionTo3x4Matrix(ColumnVector& dual_quat, Matrix& A){

	// code for this function follows math.cpp from anima-animation-sample
	//https://code.google.com/p/anima-animation-sample/source/browse/trunk/math.cpp

	ConvertQuaternionToMatrix(dual_quat, A);

	const float rx = dual_quat(2), ry = dual_quat(3), rz = dual_quat(4), rw = dual_quat(1);
	const float tx = dual_quat(6), ty = dual_quat(7), tz = dual_quat(8), tw = dual_quat(5);

	A(1, 4) = -2*tw*rx + 2*rw*tx - 2*ty*rz + 2*ry*tz;
	A(2, 4) = -2*tw*ry + 2*tx*rz - 2*rx*tz + 2*rw*ty;
	A(3, 4) = -2*tw*rz + 2*rx*ty + 2*rw*tz - 2*tx*ry;;

}


int sign(double x){
	return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}



