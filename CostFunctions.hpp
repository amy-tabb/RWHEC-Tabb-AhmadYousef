/*
 * CostFunctions1_2.hpp
 *
 *  Created on: Nov 18, 2015
 *      Author: atabb
 */

#ifndef COSTFUNCTIONS_HPP_
#define COSTFUNCTIONS_HPP_


#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
#include "Calibration2.hpp"
#include <iostream>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;

enum PARAM_TYPE {Euler, AxisAngle, Quaternion };
enum COST_TYPE {c1, c2, rp1, rp2, dh1, dh2, z, li_dq, li_kp, hirsh, shah};
enum SEPARABLE_TYPE {rotation_only, translation_only, simultaneous};

template <typename T>
void Convert6ParameterEulerAngleRepresentationIntoMatrix(const T* X, T* XM){
	T RX[9];
	ceres::EulerAnglesToRotationMatrix(X, 3, RX);

	XM[0] = RX[0];
	XM[1] = RX[1];
	XM[2] = RX[2];
	XM[3] = X[3];

	XM[4] = RX[3];
	XM[5] = RX[4];
	XM[6] = RX[5];
	XM[7] = X[4];

	XM[8] = RX[6];
	XM[9] = RX[7];
	XM[10] = RX[8];
	XM[11] = X[5];

	XM[12] = T(0);
	XM[13] = T(0);
	XM[14] = T(0);
	XM[15] = T(1);

}

template <typename T>
void Convert6ParameterAxisAngleRepresentationIntoMatrix(const T* X, T* XM){
	T RX[9];

	ceres::AngleAxisToRotationMatrix(X, RX);


	XM[0] = RX[0];
	XM[1] = RX[1];
	XM[2] = RX[2];
	XM[3] = X[3];

	XM[4] = RX[3];
	XM[5] = RX[4];
	XM[6] = RX[5];
	XM[7] = X[4];

	XM[8] = RX[6];
	XM[9] = RX[7];
	XM[10] = RX[8];
	XM[11] = X[5];

	XM[12] = T(0);
	XM[13] = T(0);
	XM[14] = T(0);
	XM[15] = T(1);

}

template <typename T>
void Convert7ParameterQuaternionRepresentationIntoMatrix(const T* X, T* XM){
	T RX[9];

	ceres::QuaternionToRotation(X, RX);


	XM[0] = RX[0];
	XM[1] = RX[1];
	XM[2] = RX[2];
	XM[3] = X[4];

	XM[4] = RX[3];
	XM[5] = RX[4];
	XM[6] = RX[5];
	XM[7] = X[5];

	XM[8] = RX[6];
	XM[9] = RX[7];
	XM[10] = RX[8];
	XM[11] = X[6];

	XM[12] = T(0);
	XM[13] = T(0);
	XM[14] = T(0);
	XM[15] = T(1);

}


template <typename T>
void MatrixMultiply(const T* X, const T* Y, T* M, int row_col){

	// assuming square matrices
	int xi, yi;
	T r;
	for (int i = 0; i < row_col; i++){
		for (int j = 0; j < row_col; j++){
			// dot product the ith row of X by the jth column of Y

			r= T(0);
			for (int index = 0; index < row_col; index++){
				xi = i*row_col + index;
				yi = index*row_col + j;

				r += X[xi]*Y[yi];
			}
			M[i*row_col + j] = r;
		}
	}
}

template <typename T>
void MatrixMultiply(const T* X, const T* Y, T* M, int row0, int col0, int row1, int col1){

	//	cout << "Args in Matrix Multiply: " << endl;
	//	cout << row0 << ", " << col0 << ", " << row1 << ", " << col1 << endl;
	//
	//	cout << "First matrix " << endl;
	//	PrintMatrix(X, row0, col0);
	//
	//	cout << "Second matrix " << endl;
	//	PrintMatrix(Y, row1, col1);


	// not assuming square matrices
	if (col0 != row1){
		cout << "Wrong args sent to Matrix Multiply: " << endl;
		cout << row0 << ", " << col0 << ", " << row1 << ", " << col1 << endl;
	}

	int xi, yi;
	T r;
	for (int i = 0; i < row0; i++){
		for (int j = 0; j < col1; j++){
			// dot product the ith row of X by the jth column of Y, results in the
			//cout << "i, j " << i << ", " << j << endl;

			r= T(0);
			for (int index = 0; index < col0; index++){
				xi = i*col0 + index; // walk across the row
				yi = index*col1 + j; // walk down the columm

				r += X[xi]*Y[yi];

				//cout << "Mult " << xi << " from first by " << yi << " from second" << endl;
			}
			//cout << "Result stored in " << i*col1 + j << endl;
			//char ch; cin >> ch;
			M[i*col1 + j] = r;
		}
	}
}

template <typename T>
void PrintMatrix(const T* X, int row0, int col0){

	for (int i = 0, index = 0; i < row0; i++){
		for (int j = 0; j < col0; j++, index++){
			cout << X[index] << " ";
		}
		cout << endl;
	}

}



// one camera for now.
struct CF1_2 {
	CF1_2(double* A, double* B, PARAM_TYPE param_type, COST_TYPE cost_type): A(A), B(B), param_type(param_type), cost_type(cost_type){
	}

	template <typename T>
	bool operator()(const T* const X,
			const T* const Z,
			T* residuals) const {
		// X and Z are 6 parameters each, and our unknowns
		// need to represent these each as matrices

		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.

		T XM[16];
		T ZM[16];

		switch (param_type){
		case Euler: {
			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case AxisAngle: {
			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case Quaternion: {
			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
		}	break;
		}

		T term0[16];
		T term1[16];

		// ||AX-ZB||
		// have to convert to T
		T AT[16];
		T BT[16];
		for (int i = 0; i < 16; i++){
			AT[i] = T(A[i]);
			BT[i] = T(B[i]);
		}


		switch (cost_type){
		case c1: {
			MatrixMultiply(AT, XM, term0, 4);
			MatrixMultiply(ZM, BT, term1, 4);


			for (int i = 0; i < 12; i++){
				residuals[i] = term1[i] - term0[i];
			}
		} break;
		case c2: {
			MatrixMultiply(ZM, BT, term0, 4);
			MatrixMultiply(term0, XM, term1, 4);

			for (int i = 0; i < 12; i++){
				residuals[i] = AT[i] - term1[i];
			}

		}break;
		default: {
			cout << "Cost type " << cost_type << "  not supported in CF1_2 " << endl;
		}

		}

		return true;
	}

	static ceres::CostFunction* Create(double* A,
			double* B, PARAM_TYPE param_type, COST_TYPE cost_type) {
		return (new ceres::AutoDiffCostFunction<CF1_2, 12, 7, 7>(
				new CF1_2(&A[0], &B[0], param_type, cost_type)));
	}

	double* A;
	double* B;
	PARAM_TYPE param_type;
	COST_TYPE cost_type;
};


struct CF1_2_multi {
	CF1_2_multi(double* A, double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras, int this_camera, double* weighting):
		A(A), B(B), param_type(param_type), cost_type(cost_type), number_cameras(number_cameras), this_camera(this_camera), weighting(weighting){
	}

	template <typename T>
	//	bool operator()(const T* const X,
	//			const T* const Z,
	//			T* residuals) const {
	bool operator()(T const* const* parameters,
			T* residuals) const {
		// X and Z are 6 parameters each, and our unknowns
		// need to represent these each as matrices

		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.

		T XM[16];
		T ZM[16];

		T X[7];
		T Z[7];

		T w = T(sqrt(*weighting));
		//cout << "Parameters for " << this_camera << endl;

		for (int i = 0; i < 7; i++){
			Z[i] = T(*parameters[(this_camera + 1)*7 + i]);
			X[i] = T(*parameters[0*7 + i]);
			//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
		}

		switch (param_type){
		case Euler: {
			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case AxisAngle: {
			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case Quaternion: {
			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
		}	break;
		}

		T term0[16];
		T term1[16];

		// ||AX-ZB||
		// have to convert to T
		T AT[16];
		T BT[16];
		for (int i = 0; i < 16; i++){
			AT[i] = T(A[i]);
			BT[i] = T(B[i]);
		}


		switch (cost_type){
		case c1: {
			MatrixMultiply(AT, XM, term0, 4);
			MatrixMultiply(ZM, BT, term1, 4);


			for (int i = 0; i < 12; i++){
				residuals[i] = w*(term1[i] - term0[i]);
			}
		} break;
		case c2: {
			MatrixMultiply(ZM, BT, term0, 4);
			MatrixMultiply(term0, XM, term1, 4);

			for (int i = 0; i < 12; i++){
				residuals[i] = (AT[i] - term1[i]);
			}

		}break;
		default: {
			cout << "Cost type " << cost_type << "  not supported in CF1_2 " << endl;
		}

		}

		return true;
	}

	//	static ceres::CostFunction* Create(double* A,
	//			double* B, PARAM_TYPE param_type, COST_TYPE cost_type) {
	//		return (new ceres::AutoDiffCostFunction<CF1_2, 12, 7, 7>(
	//				new CF1_2(&A[0], &B[0], param_type, cost_type)));
	//	}

	static ceres::CostFunction* Create(double* A,
			double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras, int this_camera, double* weighting) {


		ceres::DynamicAutoDiffCostFunction<CF1_2_multi, 7>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<CF1_2_multi, 7>(
						new CF1_2_multi(A, B, param_type, cost_type, number_cameras, this_camera, weighting));


		for (int i = 0; i < 7*(number_cameras + 1); i++){
			// one block for each X and the one Z
			cost_function->AddParameterBlock(1);
		}
		cost_function->SetNumResiduals(12);

		return cost_function;
	}

	double* A;
	double* B;
	PARAM_TYPE param_type;
	COST_TYPE cost_type;
	int number_cameras;
	int this_camera;
	double* weighting;
};


struct CF1_2_multi_extended {
	CF1_2_multi_extended(double* A, double* B, double* param_copy, PARAM_TYPE param_type, COST_TYPE cost_type, SEPARABLE_TYPE sep_type,  int number_cameras, int this_camera, double* weighting):
		A(A), B(B), param_copy(param_copy), param_type(param_type), cost_type(cost_type), sep_type(sep_type), number_cameras(number_cameras), this_camera(this_camera), weighting(weighting){
	}

	template <typename T>
	//	bool operator()(const T* const X,
	//			const T* const Z,
	//			T* residuals) const {
	bool operator()(T const* const* parameters,
			T* residuals) const {
		// X and Z are 6 parameters each, and our unknowns
		// need to represent these each as matrices

		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.

		T XM[16];
		T ZM[16];

		T X[7];
		T Z[7];

		T w = T(sqrt(*weighting));
		//cout << "Parameters for " << this_camera << endl;

		switch (sep_type){
		case rotation_only: {
			for (int i = 0; i < 7; i++){
				Z[i] = T(*parameters[(this_camera + 1)*7 + i]);
				X[i] = T(*parameters[0*7 + i]);
				//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
			}
		}; break;
		case translation_only: {
			if (param_type == Quaternion){
				for (int i = 0; i < 7; i++){
					if (i < 4){
						Z[i] = T(param_copy[(this_camera + 1)*7 + i]);
						X[i] = T(param_copy[0*7 + i]);
					}	else {
						Z[i] = T(*parameters[(this_camera + 1)*7 + i]);
						X[i] = T(*parameters[0*7 + i]);
					}
					//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
				}

			}	else {
				for (int i = 0; i < 7; i++){
					if (i < 3){
						Z[i] = T(param_copy[(this_camera + 1)*7 + i]);
						X[i] = T(param_copy[0*7 + i]);
					}	else {
						Z[i] = T(*parameters[(this_camera + 1)*7 + i]);
						X[i] = T(*parameters[0*7 + i]);
					}
					//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
				}
			}

		} break;
		case simultaneous: {
			cout << "We don't let simultaneous currently use this function .... " << endl;
			exit(1);
		} break;



		}
		//		for (int i = 0; i < 7; i++){
		//			Z[i] = T(*parameters[(this_camera + 1)*7 + i]);
		//			X[i] = T(*parameters[0*7 + i]);
		//			//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
		//		}

		switch (param_type){
		case Euler: {
			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case AxisAngle: {
			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case Quaternion: {
			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
		}	break;
		}

		T term0[16];
		T term1[16];

		// ||AX-ZB||
		// have to convert to T
		T AT[16];
		T BT[16];
		for (int i = 0; i < 16; i++){
			AT[i] = T(A[i]);
			BT[i] = T(B[i]);
		}


		switch (cost_type){
		case c1: {
			MatrixMultiply(AT, XM, term0, 4);
			MatrixMultiply(ZM, BT, term1, 4);

			switch (sep_type){
			case rotation_only: {
				for (int i = 0; i < 12; i++){
					if ((i+ 1) % 4 == 0){
						residuals[i] = T(0);
					}	else {
						residuals[i] = w*(term1[i] - term0[i]);
					}
				}
			} break;
			case translation_only: {
				for (int i = 0; i < 12; i++){
					if ((i+ 1) % 4 == 0){
						residuals[i] = w*(term1[i] - term0[i]);
					}	else {
						residuals[i] = T(0);
					}
				}

			} break;
			default: {
				for (int i = 0; i < 12; i++){
					residuals[i] = w*(term1[i] - term0[i]);
				}
			}
			}

		} break;
		case c2: {
			MatrixMultiply(ZM, BT, term0, 4);
			MatrixMultiply(term0, XM, term1, 4);


			switch (sep_type){
			case rotation_only: {
				for (int i = 0; i < 12; i++){
					if ((i+ 1) % 4 == 0){
						residuals[i] = T(0);
					}	else {
						residuals[i] = w*(AT[i] - term1[i]);
					}
				}
			} break;
			case translation_only: {
				for (int i = 0; i < 12; i++){
					if ((i+ 1) % 4 == 0){
						residuals[i] = w*(AT[i] - term1[i]);
					}	else {
						residuals[i] = T(0);
					}
				}

			} break;
			default: {
				for (int i = 0; i < 12; i++){
					residuals[i] = w*(AT[i] - term1[i]);
				}
			}
			}

		}break;
		default: {
			cout << "Cost type " << cost_type << "  not supported in CF1_2 " << endl;
		}

		}

		return true;
	}

	//	static ceres::CostFunction* Create(double* A,
	//			double* B, PARAM_TYPE param_type, COST_TYPE cost_type) {
	//		return (new ceres::AutoDiffCostFunction<CF1_2, 12, 7, 7>(
	//				new CF1_2(&A[0], &B[0], param_type, cost_type)));
	//	}

	static ceres::CostFunction* Create(double* A,
			double* B, double* param_copy, PARAM_TYPE param_type, COST_TYPE cost_type, SEPARABLE_TYPE sep_type, int number_cameras, int this_camera, double* weighting) {


		ceres::DynamicAutoDiffCostFunction<CF1_2_multi_extended, 7>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<CF1_2_multi_extended, 7>(
						new CF1_2_multi_extended(A, B, param_copy, param_type, cost_type, sep_type, number_cameras, this_camera, weighting));


		for (int i = 0; i < 7*(number_cameras + 1); i++){
			// one block for each X and the one Z
			cost_function->AddParameterBlock(1);
		}


		cost_function->SetNumResiduals(12);

		return cost_function;
	}

	double* A;
	double* B;
	double* param_copy;
	PARAM_TYPE param_type;
	COST_TYPE cost_type;
	SEPARABLE_TYPE sep_type;
	int number_cameras;
	int this_camera;
	double* weighting;
};


// create is for one point only ....
//struct RP1_2_multi {
//	RP1_2_multi(double* camera_parameters, double* B, double* twoDpoints, double* threeDpoints, PARAM_TYPE param_type, COST_TYPE cost_type, int number_points,
//			int number_cameras, int this_camera, double* weighting):
//				camera_parameters(camera_parameters), B(B), twoDpoints(twoDpoints), threeDpoints(threeDpoints),
//				param_type(param_type), cost_type(cost_type), number_points(number_points),
//				number_cameras(number_cameras), this_camera(this_camera), weighting(weighting){
//	}
//
//	template <typename T>
//	//	bool operator()(const T* const X,
//	//			const T* const Z,
//	//			T* residuals) const {
//	bool operator()(T const* const* parameters,
//			T* residuals) const {
//		// X and Z are 7 parameters each, and our unknowns
//		// need to represent these each as matrices
//		// parameters: X0 (7) cam_parameters(12) X1(7) cam_parameters(12) Z(7)
//
//		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
//		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
//		char ch;
//		//cout << "Line 767 " << endl; //cin >> ch;
//		T XM[16];
//		T ZM[16];
//
//		T X[7];
//		T Z[7];
//
//		T w = T(sqrt(*weighting));
//		//cout << "Parameters for " << this_camera << endl;
//
//		for (int i = 0; i < 7; i++){
//			Z[i] = T(*parameters[number_cameras*19 + i]);
//			X[i] = T(*parameters[this_camera*19 + i]);
//		}
//
//		switch (param_type){
//		case Euler: {
//			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
//			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
//		}	break;
//		case AxisAngle: {
//			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
//			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
//		}	break;
//		case Quaternion: {
//			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
//			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
//		}	break;
//		}
//
//		T term0[16];
//
//
//		// ||AX-ZB||
//		// have to convert to T
//		T A_hat_T[16];
//		T BT[16];
//		T M[16];
//		T K[9];
//		T k1, k2, p1, p2, k3, k4, k5, k6;
//		T r_sqr;
//
//		//cout << "Line 807 " << endl;
//		//cin >> ch;
//
//		T Xp[4];
//		T xp[3];
//		T xpp[3];
//
//		for (int i = 0; i < 16; i++){
//
//			BT[i] = T(B[i]);
//		}
//
//		for (int i = 0; i < 9; i++){
//			K[i] = T(0);
//		}
//
//		K[8] = T(1);
//
//		// set up A_hat
//		MatrixMultiply(ZM, BT, term0, 4);
//		MatrixMultiply(term0, XM, A_hat_T, 4);
//
//
//		// copy over parameters
//		switch (cost_type){
//		case rp1: {
//			// don't use the parameter version for camera cali -- this amtrix is sent all set up for use ....
//			K[0] = T(camera_parameters[0]);
//			K[2] = T(camera_parameters[1]);
//			K[4] = T(camera_parameters[2]);
//			K[5] = T(camera_parameters[3]);
//			k1 = T(camera_parameters[4]);
//			k2 = T(camera_parameters[5]);
//			p1 = T(camera_parameters[6]);
//			p2 = T(camera_parameters[7]);
//			k3 = T(camera_parameters[8]);
//			k4 = T(camera_parameters[9]);
//			k5 = T(camera_parameters[10]);
//			k6 = T(camera_parameters[11]);
//		} break;
//		case rp2:{
//			K[0] = *parameters[this_camera*19 + 7];
//			K[2] = *parameters[this_camera*19 + 8];
//			K[4] = *parameters[this_camera*19 + 9];
//			K[5] = *parameters[this_camera*19 + 10];
//			k1 = *parameters[this_camera*19 + 11];
//			k2 = *parameters[this_camera*19 + 12];
//			p1 = *parameters[this_camera*19 + 13];
//			p2 = *parameters[this_camera*19 + 14];
//			k3 = *parameters[this_camera*19 + 15];
//			k4 = *parameters[this_camera*19 + 16];
//			k5 = *parameters[this_camera*19 + 17];
//			k6 = *parameters[this_camera*19 + 18];
//		} break;
//		default: {
//
//		}
//
//		}
//
//		for (int j = 0; j < 3; j++){
//			Xp[j] = T(threeDpoints[j]);
//		}
//		Xp[3] = T(1);
//
//		MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);
//
//		xp[0] = xp[0]/xp[2];
//		xp[1] = xp[1]/xp[2];
//
//		r_sqr= xp[0]*xp[0] + xp[1]*xp[1];
//
//		T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
//		T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);
//
//		xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
//		xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);
//
//		T predicted_x = (xpp[0]*K[0] + K[2]);
//		T predicted_y = (xpp[1]*K[4] + K[5]);
//
//		// TODO out put values and see what is going on here ....
////		cout << "XW" << endl;
////		PrintMatrix(Xp, 4, 1);
////
////		cout << "xp" << endl;
////		PrintMatrix(xp, 3, 1);
////
////		cout << "Predicted x " << predicted_x << endl;
////		cout << "Predicted y " << predicted_y << endl;
////
////		cout << "Obs0 " << twoDpoints[0] << endl;
////		cout << "Obs1 " << twoDpoints[1] << endl;
//
//		residuals[0] = w*(predicted_x - T(twoDpoints[0]));
//		residuals[1] = w*(predicted_y - T(twoDpoints[1]));
//
//		//cin >> ch;
//		return true;
//	}
//
//	static ceres::CostFunction* Create(double* camera_parameters, double* B, double* twoDpoints, double* threeDpoints,
//			PARAM_TYPE param_type, COST_TYPE cost_type, int number_points, int number_cameras, int this_camera, double* weighting) {
//
//
//		ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 10>* cost_function =
//				new ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 10>(
//						new RP1_2_multi(camera_parameters, B, twoDpoints, threeDpoints, param_type, cost_type, number_points, number_cameras, this_camera, weighting));
//
//
//		for (int i = 0; i <19*(number_cameras) + 7; i++){
//			cost_function->AddParameterBlock(1);
//		}
//		cost_function->SetNumResiduals(2);
//
//		return cost_function;
//	}
//
//	double* camera_parameters;
//	double* B;
//	double* twoDpoints;
//	double* threeDpoints;
//
//	PARAM_TYPE param_type;
//	COST_TYPE cost_type;
//	int number_points;
//	int number_cameras;
//	int this_camera;
//	double* weighting;
//};

struct RP1_2_multi {
	RP1_2_multi(double* camera_parameters, double* B, double* twoDpoints, double* threeDpoints, PARAM_TYPE param_type, COST_TYPE cost_type, int number_points,
			int number_cameras, int this_camera, double* weighting):
				camera_parameters(camera_parameters), B(B), twoDpoints(twoDpoints), threeDpoints(threeDpoints),
				param_type(param_type), cost_type(cost_type), number_points(number_points),
				number_cameras(number_cameras), this_camera(this_camera), weighting(weighting){
	}

	template <typename T>
	//	bool operator()(const T* const X,
	//			const T* const Z,
	//			T* residuals) const {
	bool operator()(T const* const* parameters,
			T* residuals) const {
		// X and Z are 7 parameters each, and our unknowns
		// need to represent these each as matrices
		// parameters: X0 (7) cam_parameters(12) X1(7) cam_parameters(12) Z(7)

		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
		char ch;
		//cout << "Line 767 " << endl; //cin >> ch;
		T XM[16];
		T ZM[16];

		T X[7];
		T Z[7];

		T w = T(sqrt(*weighting));
		//cout << "Parameters for " << this_camera << endl;

		//		for (int i = 0; i < 7; i++){
		//			Z[i] = T(*parameters[number_cameras*19 + i]);
		//			X[i] = T(*parameters[this_camera*19 + i]);
		//		}

		for (int i = 0; i < 7; i++){
			X[i] = T(*parameters[0*7 + i]);
			Z[i] = T(*parameters[7 + (this_camera)*19 + i]);

			//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
		}

		switch (param_type){
		case Euler: {
			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case AxisAngle: {
			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case Quaternion: {
			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
		}	break;
		}

		T term0[16];


		// ||AX-ZB||
		// have to convert to T
		T A_hat_T[16];
		T BT[16];
		T M[16];
		T K[9];
		T k1, k2, p1, p2, k3, k4, k5, k6;
		T r_sqr;

		//cout << "Line 807 " << endl;
		//cin >> ch;

		T Xp[4];
		T xp[3];
		T xpp[3];

		for (int i = 0; i < 16; i++){

			BT[i] = T(B[i]);
		}

		for (int i = 0; i < 9; i++){
			K[i] = T(0);
		}

		K[8] = T(1);

		// set up A_hat
		MatrixMultiply(ZM, BT, term0, 4);
		MatrixMultiply(term0, XM, A_hat_T, 4);


		// copy over parameters
		switch (cost_type){
		case rp1: {
			// don't use the parameter version for camera cali -- this amtrix is sent all set up for use ....
			K[0] = T(camera_parameters[0]);
			K[2] = T(camera_parameters[1]);
			K[4] = T(camera_parameters[2]);
			K[5] = T(camera_parameters[3]);
			k1 = T(camera_parameters[4]);
			k2 = T(camera_parameters[5]);
			p1 = T(camera_parameters[6]);
			p2 = T(camera_parameters[7]);
			k3 = T(camera_parameters[8]);
			k4 = T(camera_parameters[9]);
			k5 = T(camera_parameters[10]);
			k6 = T(camera_parameters[11]);
		} break;
		case rp2:{
			K[0] = *parameters[7 + (this_camera)*19 + 7];
			K[2] = *parameters[7 + (this_camera)*19 + 8];
			K[4] = *parameters[7 + (this_camera)*19 + 9];
			K[5] = *parameters[7 + (this_camera)*19 + 10];
			k1 = *parameters[7 + (this_camera)*19 + 11];
			k2 = *parameters[7 + (this_camera)*19 + 12];
			p1 = *parameters[7 + (this_camera)*19 + 13];
			p2 = *parameters[7 + (this_camera)*19 + 14];
			k3 = *parameters[7 + (this_camera)*19 + 15];
			k4 = *parameters[7 + (this_camera)*19 + 16];
			k5 = *parameters[7 + (this_camera)*19 + 17];
			k6 = *parameters[7 + (this_camera)*19 + 18];
		} break;
		default: {

		}

		}

		for (int j = 0; j < 3; j++){
			Xp[j] = T(threeDpoints[j]);
		}
		Xp[3] = T(1);

		MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);

		xp[0] = xp[0]/xp[2];
		xp[1] = xp[1]/xp[2];

		r_sqr= xp[0]*xp[0] + xp[1]*xp[1];

		T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
		T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);

		xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
		xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);

		T predicted_x = (xpp[0]*K[0] + K[2]);
		T predicted_y = (xpp[1]*K[4] + K[5]);

		// TODO out put values and see what is going on here ....
		//		cout << "XW" << endl;
		//		PrintMatrix(Xp, 4, 1);
		//
		//		cout << "xp" << endl;
		//		PrintMatrix(xp, 3, 1);
		//
		//		cout << "Predicted x " << predicted_x << endl;
		//		cout << "Predicted y " << predicted_y << endl;
		//
		//		cout << "Obs0 " << twoDpoints[0] << endl;
		//		cout << "Obs1 " << twoDpoints[1] << endl;

		residuals[0] = w*(predicted_x - T(twoDpoints[0]));
		residuals[1] = w*(predicted_y - T(twoDpoints[1]));

		//cin >> ch;
		return true;
	}

	static ceres::CostFunction* Create(double* camera_parameters, double* B, double* twoDpoints, double* threeDpoints,
			PARAM_TYPE param_type, COST_TYPE cost_type, int number_points, int number_cameras, int this_camera, double* weighting) {


		ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 10>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 10>(
						new RP1_2_multi(camera_parameters, B, twoDpoints, threeDpoints, param_type, cost_type, number_points, number_cameras, this_camera, weighting));


		for (int i = 0; i <19*(number_cameras) + 7; i++){
			cost_function->AddParameterBlock(1);
		}
		cost_function->SetNumResiduals(2);

		return cost_function;
	}

	double* camera_parameters;
	double* B;
	double* twoDpoints;
	double* threeDpoints;

	PARAM_TYPE param_type;
	COST_TYPE cost_type;
	int number_points;
	int number_cameras;
	int this_camera;
	double* weighting;
};


struct RP1_2_multi_extended {
	RP1_2_multi_extended(double* camera_parameters, double* B, double* param_copy, double* twoDpoints, double* threeDpoints,
			PARAM_TYPE param_type, COST_TYPE cost_type, SEPARABLE_TYPE sep_type, int number_points,
			int number_cameras, int this_camera, double* weighting):
				camera_parameters(camera_parameters), B(B), param_copy(param_copy), twoDpoints(twoDpoints), threeDpoints(threeDpoints),
				param_type(param_type), cost_type(cost_type), sep_type(sep_type), number_points(number_points),
				number_cameras(number_cameras), this_camera(this_camera), weighting(weighting){
	}

	template <typename T>
	//	bool operator()(const T* const X,
	//			const T* const Z,
	//			T* residuals) const {
	bool operator()(T const* const* parameters,
			T* residuals) const {
		// X and Z are 7 parameters each, and our unknowns
		// need to represent these each as matrices
		// parameters: X0 (7) cam_parameters(12) X1(7) cam_parameters(12) Z(7)

		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
		char ch;
		//cout << "Line 767 " << endl; //cin >> ch;
		T XM[16];
		T ZM[16];

		T X[7];
		T Z[7];

		T w = T(sqrt(*weighting));
		//cout << "Parameters for " << this_camera << endl;

		//		for (int i = 0; i < 7; i++){
		//			Z[i] = T(*parameters[number_cameras*19 + i]);
		//			X[i] = T(*parameters[this_camera*19 + i]);
		//		}

		int break_point = 3;

		if (param_type == Quaternion){
			break_point = 4;
		}
		switch (sep_type){
		case rotation_only: {
			for (int i = 0; i < 7; i++){
				if (i < break_point){
					X[i] = T(*parameters[0*7 + i]);
					Z[i] = T(*parameters[7 + (this_camera)*19 + i]);
				}	else {
					Z[i] = T(param_copy[(this_camera + 1)*7 + i]);
					X[i] = T(param_copy[0*7 + i]);
				}
			}
		}; break;
		case translation_only: {
			for (int i = 0; i < 7; i++){
				if (i < break_point){
					Z[i] = T(param_copy[(this_camera + 1)*7 + i]);
					X[i] = T(param_copy[0*7 + i]);

				}	else {
					X[i] = T(*parameters[0*7 + i]);
					Z[i] = T(*parameters[7 + (this_camera)*19 + i]);
				}
			}
		} break;
		case simultaneous: {
			for (int i = 0; i < 7; i++){
				X[i] = T(*parameters[0*7 + i]);
				Z[i] = T(*parameters[7 + (this_camera)*19 + i]);
			}
		} break;
		}

		switch (param_type){
		case Euler: {
			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case AxisAngle: {
			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
		}	break;
		case Quaternion: {
			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
		}	break;
		}

		T term0[16];


		// ||AX-ZB||
		// have to convert to T
		T A_hat_T[16];
		T BT[16];
		T M[16];
		T K[9];
		T k1, k2, p1, p2, k3, k4, k5, k6;
		T r_sqr;

		//cout << "Line 807 " << endl;
		//cin >> ch;

		T Xp[4];
		T xp[3];
		T xpp[3];

		for (int i = 0; i < 16; i++){

			BT[i] = T(B[i]);
		}

		for (int i = 0; i < 9; i++){
			K[i] = T(0);
		}

		K[8] = T(1);

		// set up A_hat
		MatrixMultiply(ZM, BT, term0, 4);
		MatrixMultiply(term0, XM, A_hat_T, 4);


		// copy over parameters
		switch (cost_type){
		case rp1: {
			// don't use the parameter version for camera cali -- this amtrix is sent all set up for use ....
			K[0] = T(camera_parameters[0]);
			K[2] = T(camera_parameters[1]);
			K[4] = T(camera_parameters[2]);
			K[5] = T(camera_parameters[3]);
			k1 = T(camera_parameters[4]);
			k2 = T(camera_parameters[5]);
			p1 = T(camera_parameters[6]);
			p2 = T(camera_parameters[7]);
			k3 = T(camera_parameters[8]);
			k4 = T(camera_parameters[9]);
			k5 = T(camera_parameters[10]);
			k6 = T(camera_parameters[11]);
		} break;
		case rp2:{
			K[0] = *parameters[7 + (this_camera)*19 + 7];
			K[2] = *parameters[7 + (this_camera)*19 + 8];
			K[4] = *parameters[7 + (this_camera)*19 + 9];
			K[5] = *parameters[7 + (this_camera)*19 + 10];
			k1 = *parameters[7 + (this_camera)*19 + 11];
			k2 = *parameters[7 + (this_camera)*19 + 12];
			p1 = *parameters[7 + (this_camera)*19 + 13];
			p2 = *parameters[7 + (this_camera)*19 + 14];
			k3 = *parameters[7 + (this_camera)*19 + 15];
			k4 = *parameters[7 + (this_camera)*19 + 16];
			k5 = *parameters[7 + (this_camera)*19 + 17];
			k6 = *parameters[7 + (this_camera)*19 + 18];
		} break;
		default: {

		}

		}

		for (int j = 0; j < 3; j++){
			Xp[j] = T(threeDpoints[j]);
		}
		Xp[3] = T(1);

		MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);

		xp[0] = xp[0]/xp[2];
		xp[1] = xp[1]/xp[2];

		r_sqr= xp[0]*xp[0] + xp[1]*xp[1];

		T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
		T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);

		xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
		xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);

		T predicted_x = (xpp[0]*K[0] + K[2]);
		T predicted_y = (xpp[1]*K[4] + K[5]);

		// TODO out put values and see what is going on here ....
		//		cout << "XW" << endl;
		//		PrintMatrix(Xp, 4, 1);
		//
		//		cout << "xp" << endl;
		//		PrintMatrix(xp, 3, 1);
		//
		//		cout << "Predicted x " << predicted_x << endl;
		//		cout << "Predicted y " << predicted_y << endl;
		//
		//		cout << "Obs0 " << twoDpoints[0] << endl;
		//		cout << "Obs1 " << twoDpoints[1] << endl;

		residuals[0] = w*(predicted_x - T(twoDpoints[0]));
		residuals[1] = w*(predicted_y - T(twoDpoints[1]));

		//cin >> ch;
		return true;
	}

	static ceres::CostFunction* Create(double* camera_parameters, double* B, double* param_copy, double* twoDpoints, double* threeDpoints,
			PARAM_TYPE param_type, COST_TYPE cost_type, SEPARABLE_TYPE sep_type, int number_points, int number_cameras, int this_camera, double* weighting) {


		ceres::DynamicAutoDiffCostFunction<RP1_2_multi_extended, 10>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<RP1_2_multi_extended, 10>(
						new RP1_2_multi_extended(camera_parameters, B, param_copy, twoDpoints, threeDpoints, param_type, cost_type, sep_type, number_points, number_cameras, this_camera, weighting));


		for (int i = 0; i <19*(number_cameras) + 7; i++){
			cost_function->AddParameterBlock(1);
		}
		cost_function->SetNumResiduals(2);

		return cost_function;
	}

	double* camera_parameters;
	double* B;
	double* param_copy;
	double* twoDpoints;
	double* threeDpoints;

	PARAM_TYPE param_type;
	COST_TYPE cost_type;
	SEPARABLE_TYPE sep_type;
	int number_points;
	int number_cameras;
	int this_camera;
	double* weighting;
};


//// we need the camera parameters, transformation matrices (one per camera), and twoDpoints for each camera that can see the pattern.
//	// for now, solve all together as one big cost function
//	// result is the three d points for the pattern.
//	double* camera_parameters;
//	double* transformation_matrices;
//	double* twoDpoints;
//
//	int number_points;
//	int number_cameras;

// this is all per camera, we don't need to send out all items at once ....
struct ReconstructX {
	ReconstructX(double* camera_parameters, double* transformation_matrix, double* twoDpoints, int number_points, bool individual, int this_point):
		camera_parameters(camera_parameters), transformation_matrix(transformation_matrix), twoDpoints(twoDpoints),
		number_points(number_points), individual(individual), this_point(this_point){
	}

	template <typename T>
	//	bool operator()(const T* const X,
	//			const T* const Z,
	//			T* residuals) const {
	bool operator()(T const* const* parameters,
			T* residuals) const {
		// the parameters are the threed points now

		char ch;
		// have to convert to T
		T A_hat_T[16];
		//T BT[16];
		T M[16];
		T K[9];
		T k1, k2, p1, p2, k3, k4, k5, k6;
		T r_sqr;
		T Xp[4];
		T xp[3];
		T xpp[3];


		// read in transformation matric
		for (int i = 0; i < 16; i++){
			A_hat_T[i] = T(transformation_matrix[i]);
		}

		// Read in camera calibration matrix
		for (int i = 0; i < 9; i++){
			K[i] = T(0);
		}
		K[8] = T(1);

		// don't use the parameter version for camera cali -- this amtrix is sent all set up for use ....
		K[0] = T(camera_parameters[0]);
		K[2] = T(camera_parameters[1]);
		K[4] = T(camera_parameters[2]);
		K[5] = T(camera_parameters[3]);
		k1 = T(camera_parameters[4]);
		k2 = T(camera_parameters[5]);
		p1 = T(camera_parameters[6]);
		p2 = T(camera_parameters[7]);
		k3 = T(camera_parameters[8]);
		k4 = T(camera_parameters[9]);
		k5 = T(camera_parameters[10]);
		k6 = T(camera_parameters[11]);


		if (!individual){
			for (int i = 0; i < number_points; i++){
				for (int j = 0; j < 3; j++){
					Xp[j] = T(*parameters[3*i + j]);
				}
				Xp[3] = T(1);

				MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);

				xp[0] = xp[0]/xp[2];
				xp[1] = xp[1]/xp[2];

				r_sqr= xp[0]*xp[0] + xp[1]*xp[1];

				T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
				T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);

				xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
				xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);

				T predicted_x = (xpp[0]*K[0] + K[2]);
				T predicted_y = (xpp[1]*K[4] + K[5]);

				//if (i < 1){
				residuals[2*i] = (predicted_x - T(twoDpoints[2*i + 0]));
				residuals[2*i + 1] = (predicted_y - T(twoDpoints[2*i + 1]));
				//			}	else {
				//				residuals[2*i] = T(0);
				//				residuals[2*i + 1] = T(0);
				//			}
			}
		}	else {
			int i = this_point;

			for (int j = 0; j < 3; j++){
				Xp[j] = T(*parameters[j]);
			}
			Xp[3] = T(1);

			MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);

			xp[0] = xp[0]/xp[2];
			xp[1] = xp[1]/xp[2];

			r_sqr= xp[0]*xp[0] + xp[1]*xp[1];

			T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
			T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);

			xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
			xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);

			T predicted_x = (xpp[0]*K[0] + K[2]);
			T predicted_y = (xpp[1]*K[4] + K[5]);

			residuals[0] = (predicted_x - T(twoDpoints[2*i + 0]));
			residuals[1] = (predicted_y - T(twoDpoints[2*i + 1]));
		}

		//cin >> ch;
		return true;
	}

	static ceres::CostFunction* Create(double* camera_parameters, double* transformation_matrix, double* twoDpoints, int number_points) {


		ceres::DynamicAutoDiffCostFunction<ReconstructX, 3>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<ReconstructX, 3>(
						new ReconstructX(camera_parameters, transformation_matrix, twoDpoints, number_points, false, 0 ));


		for (int i = 0; i <3*number_points; i++){
			cost_function->AddParameterBlock(1);
		}
		cost_function->SetNumResiduals(2*number_points);

		return cost_function;
	}

	static ceres::CostFunction* CreateID(double* camera_parameters, double* transformation_matrix, double* twoDpoints, int number_points, int this_point) {


		ceres::DynamicAutoDiffCostFunction<ReconstructX, 3>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<ReconstructX, 3>(
						new ReconstructX(camera_parameters, transformation_matrix, twoDpoints, number_points, true, this_point));


		for (int i = 0; i <3; i++){
			cost_function->AddParameterBlock(1);
		}
		cost_function->SetNumResiduals(2);

		return cost_function;
	}

	// we need the camera parameters, transformation matrices (one per camera), and twoDpoints for each camera that can see the pattern.
	// for now, solve all together as one big cost function
	// result is the three d points for the pattern.
	double* camera_parameters;
	double* transformation_matrix;
	double* twoDpoints;

	int number_points;
	bool individual;
	int this_point;

};

void CF1_2_one_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, double* z, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void CF1_2_multi_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void CF1_2_multi_camera_separable(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type, SEPARABLE_TYPE sep_type);

void RP1_2_multi_camera(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, double* camera_params, double* x,
		std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void RP1_2_multi_camera_sparse(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, double* camera_params, double* x,
		std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void RP1_2_multi_camera_sparse_seperable(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, double* camera_params, double* x,
		std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type, SEPARABLE_TYPE sep_type);

void CopyFromCalibration(vector<CaliObjectOpenCV2>& COs, double* camera_params);

void CopyToCalibration(vector<CaliObjectOpenCV2>& COs, double* camera_parameters);

void ReconstructXFunction(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, Matrix& X, vector<Matrix>& Zs, double* threeDpoints, vector<double>& reprojection_errors,
		std::ofstream& out);

double ComputeSummedSquaredDistanceBetweenSets(double* set0, double* set1, int number_points);

void Initialize3DPoints(vector<CaliObjectOpenCV2>& COs, double* threeDpoints, int number_points);

void ReconstructXFunctionIndividuals(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, Matrix& X, vector<Matrix>& Zs, double* threeDpoints, vector<double>& reprojection_errors,
		std::ofstream& out);

#endif /* COSTFUNCTIONS_HPP_ */
