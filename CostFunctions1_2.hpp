/*
 * CostFunctions1_2.hpp
 *
 *  Created on: Nov 18, 2015
 *      Author: atabb
 */

#ifndef COSTFUNCTIONS1_2_HPP_
#define COSTFUNCTIONS1_2_HPP_


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
enum COST_TYPE {c1, c2, rp1, rp2 };

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

//
////struct CF1_2_general {
////	CF1_2_general(double* A, double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras):
////		A(A), B(B), param_type(param_type), cost_type(cost_type), number_cameras(number_cameras){
////	}
////
////	template <typename T>
////	bool operator()(T const* const* parameters, T* residuals) const {
////		// X and Z are 7 parameters each, and our unknowns
////		// need to represent these each as matrices
////
////		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
////		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
////		T X[7];
////		T Z[7];
////
////		T* XM = new T[16*number_cameras];
////		T* AT = new T[16*number_cameras];
////
////		for (int i = 0; i < 7; i++){
////			Z[i] = T(*parameters[number_cameras*7 + i]);
////		}
////
////		for (int i = 0; i < 16*number_cameras; i++){
////			AT[i] = T(A[i]);
////		}
////
////
////		for (int c = 0; c < number_cameras; c++){
////
////			/// HERE using X/XM only instead of the whole set .. make a new matrix rep for all of the rotation matrices.
////			for (int j = 0; j < 7; j++){
////				X[j] = T(*parameters[7*c + j]);
////			}
////
////
////
////			//T XM[16];
////			T ZM[16];
////
////			switch (param_type){
////			case Euler: {
////				Convert6ParameterEulerAngleRepresentationIntoMatrix(X, &XM[16*c]);
////				Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
////			}	break;
////			case AxisAngle: {
////				Convert6ParameterAxisAngleRepresentationIntoMatrix(X, &XM[16*c]);
////				Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
////			}	break;
////			case Quaternion: {
////				Convert7ParameterQuaternionRepresentationIntoMatrix(X, &XM[16*c]);
////				Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
////			}	break;
////			}
////
////			T term0[16];
////			T term1[16];
////
////
////			// have to convert to T
////			//T AT[16];
////			T BT[16];
////			for (int i = 0; i < 16; i++){
////				//AT[i] = T(A[16*c + i]);
////				BT[i] = T(B[i]);
////			}
////
////
////			switch (cost_type){
////			case c1: {
////				MatrixMultiply(&AT[16*c], &XM[16*c], term0, 4);
////				MatrixMultiply(ZM, BT, term1, 4);
////
////
////				for (int i = 0; i < 12; i++){
////					residuals[12*c + i] = term1[i] - term0[i];
////				}
////			} break;
////			case c2: {
////				MatrixMultiply(ZM, BT, term0, 4);
////				MatrixMultiply(term0, &XM[16*c], term1, 4);
////
////				for (int i = 0; i < 12; i++){
////					residuals[12*c + i] = AT[16*c + i] - term1[i];
////				}
////
////			}break;
////			default: {
////				cout << "Cost type " << cost_type << "  not supported in CF1_2 " << endl;
////			}
////			}
////
////		}
////
////		delete [] XM;
////		delete [] AT;
////
////		return true;
////	}
////
////	static ceres::CostFunction* Create(double* A,
////			double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras) {
////
////
////		ceres::DynamicAutoDiffCostFunction<CF1_2_general, 7>* cost_function =
////				new ceres::DynamicAutoDiffCostFunction<CF1_2_general, 7>(
////						new CF1_2_general(&A[0], &B[0], param_type, cost_type, number_cameras));
////
////		//cost_function->AddParameterBlock(7*(number_cameras + 1));
////		for (int i = 0; i < 7*(number_cameras + 1); i++){
////			// one block for each X and the one Z
////			cost_function->AddParameterBlock(1);
////		}
////		cost_function->SetNumResiduals(12*number_cameras);
////
////		return cost_function;
////	}
////
////	double* A;
////	double* B;
////	PARAM_TYPE param_type;
////	COST_TYPE cost_type;
////	int number_cameras;
////};
//
//// only add one block at a time?? next
//struct CF1_2_general {
//	CF1_2_general(double* A, double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras):
//		A(A), B(B), param_type(param_type), cost_type(cost_type), number_cameras(number_cameras){
//	}
//
//	template <typename T>
//	bool operator()(T const* const* parameters, T* residuals) const {
//		// X and Z are 7 parameters each, and our unknowns
//		// need to represent these each as matrices
//
//		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
//		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
//		T* X = new T[7*number_cameras];
//		T Z[7];
//
//		T* XM = new T[16*number_cameras];
//		T* AT = new T[16*number_cameras];
//		T* term0 = new T[16*number_cameras];
//		T* term1 = new T[16*number_cameras];
//
//		for (int i = 0; i < 7; i++){
//			Z[i] = T(*parameters[number_cameras*7 + i]);
//		}
//
//		for (int i = 0; i < 16*number_cameras; i++){
//			AT[i] = T(A[i]);
//		}
//
//		T BT[16];
//		for (int i = 0; i < 16; i++){
//			BT[i] = T(B[i]);
//		}
//
//
//
//		for (int c = 0; c < number_cameras; c++){
//
//			/// HERE using X/XM only instead of the whole set .. make a new matrix rep for all of the rotation matrices.
//			for (int j = 0; j < 7; j++){
//				X[7*c + j] = T(*parameters[7*c + j]);
//			}
//
//
//
//			//T XM[16];
//			T ZM[16];
//
//			switch (param_type){
//			case Euler: {
//				Convert6ParameterEulerAngleRepresentationIntoMatrix(&X[7*c], &XM[16*c]);
//				Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
//			}	break;
//			case AxisAngle: {
//				Convert6ParameterAxisAngleRepresentationIntoMatrix(&X[7*c], &XM[16*c]);
//				Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
//			}	break;
//			case Quaternion: {
//				Convert7ParameterQuaternionRepresentationIntoMatrix(&X[7*c], &XM[16*c]);
//				Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
//			}	break;
//			}
//
//
//
//
//			// have to convert to T
//			//T AT[16];
//
//
//			switch (cost_type){
//			case c1: {
//				MatrixMultiply(&AT[16*c], &XM[16*c], &term0[16*c], 4);
//				MatrixMultiply(ZM, BT, &term1[16*c], 4);
//
//
//				for (int i = 0; i < 12; i++){
//					residuals[12*c + i] = term1[16*c + i] - term0[16*c + i];
//				}
//			} break;
//			case c2: {
//				MatrixMultiply(ZM, BT, &term0[16*c], 4);
//				MatrixMultiply(&term0[16*c], &XM[16*c], &term1[16*c], 4);
//
//				for (int i = 0; i < 12; i++){
//					residuals[12*c + i] = AT[16*c + i] - term1[16*c + i];
//				}
//
//			}break;
//			default: {
//				cout << "Cost type " << cost_type << "  not supported in CF1_2 " << endl;
//			}
//			}
//
//		}
//
//		delete [] XM;
//		delete [] AT;
//		delete [] term0;
//		delete [] term1;
//		delete [] X;
//
//		return true;
//	}
//
//
//
//	static ceres::CostFunction* Create(double* A,
//			double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras) {
//
//
//		ceres::DynamicAutoDiffCostFunction<CF1_2_general, 7>* cost_function =
//				new ceres::DynamicAutoDiffCostFunction<CF1_2_general, 7>(
//						new CF1_2_general(&A[0], &B[0], param_type, cost_type, number_cameras));
//
//		//cost_function->AddParameterBlock(7*(number_cameras + 1));
//		for (int i = 0; i < 7*(number_cameras + 1); i++){
//			// one block for each X and the one Z
//			cost_function->AddParameterBlock(1);
//		}
//		cost_function->SetNumResiduals(12*number_cameras);
//
//		return cost_function;
//	}
//
//
//	double* A;
//	double* B;
//	PARAM_TYPE param_type;
//	COST_TYPE cost_type;
//	int number_cameras;
//};
//
//struct CF1_2_generalB {
//	CF1_2_generalB(double* A, double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras, int this_camera):
//		A(A), B(B), param_type(param_type), cost_type(cost_type), number_cameras(number_cameras), this_camera(this_camera){
//	}
//
//
//
//	template <typename T>
//	bool operator()(T const* const* parameters,
//			T* residuals) const {
//		// X and Z are 6 parameters each, and our unknowns
//		// need to represent these each as matrices
//
//		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
//		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
//
//		// A is by camera, B also by camera
//
//		T X[7];
//		T Z[7];
//		T AT[16];
//		T BT[16];
//		T XM[16];
//		T ZM[16];
//
//		//cout << "Parameters for " << this_camera << endl;
//
//		for (int i = 0; i < 7; i++){
//			Z[i] = T(*parameters[number_cameras*7 + i]);
//			X[i] = T(*parameters[this_camera*7 + i]);
//			//cout << "X, Z: " << X[i] << ", " << Z[i] << endl;
//		}
//
//		//char ch;  cin >> ch;
//
//
//		for (int i = 0; i < 16; i++){
//			AT[i] = T(A[i]);
//			BT[i] = T(B[i]);
//		}
//
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
//		T term1[16];
//
//		switch (cost_type){
//		case c1: {
//			MatrixMultiply(AT, XM, term0, 4);
//			MatrixMultiply(ZM, BT, term1, 4);
//
//
//			for (int i = 0; i < 12; i++){
//				residuals[i] = term1[i] - term0[i];
//			}
//		} break;
//		case c2: {
//			MatrixMultiply(ZM, BT, term0, 4);
//			MatrixMultiply(term0, XM, term1, 4);
//
//			for (int i = 0; i < 12; i++){
//				residuals[i] = AT[i] - term1[i];
//			}
//
//		}break;
//		default: {
//			cout << "Cost type " << cost_type << "  not supported in CF1_2 " << endl;
//		}
//
//		}
//
//		return true;
//	}
//
//	static ceres::CostFunction* Create(double* A,
//			double* B, PARAM_TYPE param_type, COST_TYPE cost_type, int number_cameras, int this_camera) {
//
//
//		ceres::DynamicAutoDiffCostFunction<CF1_2_generalB, 7>* cost_function =
//				new ceres::DynamicAutoDiffCostFunction<CF1_2_generalB, 7>(
//						new CF1_2_generalB(&A[0], &B[0], param_type, cost_type, number_cameras, this_camera));
//
//
//		for (int i = 0; i < 7*(number_cameras + 1); i++){
//			// one block for each X and the one Z
//			cost_function->AddParameterBlock(1);
//		}
//		cost_function->SetNumResiduals(12);
//
//		return cost_function;
//	}
//
//	double* A;
//	double* B;
//	PARAM_TYPE param_type;
//	COST_TYPE cost_type;
//	int number_cameras;
//	int this_camera;
//};



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
			Z[i] = T(*parameters[number_cameras*7 + i]);
			X[i] = T(*parameters[this_camera*7 + i]);
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
				residuals[i] = AT[i] - term1[i];
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
//		cout << "Line 767 " << endl; //cin >> ch;
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
//		T* r_sqr = new T[number_points];
//
//		cout << "Line 807 " << endl;
//		//cin >> ch;
//
//		T* Xp = new T[number_points*4];
//		T* xp = new T[number_points*3];
//		T* xpp = new T[number_points*3];
//
//		for (int i = 0; i < 16; i++){
//			K[i] = T(0);
//			BT[i] = T(B[i]);
//		}
//		K[8] = T(1);
//		//K[15] = T(1);
//
//		// set up A_hat
//		MatrixMultiply(ZM, BT, term0, 4);
//		MatrixMultiply(term0, XM, A_hat_T, 4);
//
//
//
//		// copy over parameters
//		switch (cost_type){
//		case rp1: {
//			// don't use the parameter version for camera cali
//			K[0] = T(camera_parameters[12*this_camera]);
//			K[2] = T(camera_parameters[12*this_camera + 1]);
//			K[4] = T(camera_parameters[12*this_camera + 2]);
//			K[5] = T(camera_parameters[12*this_camera + 3]);
//			k1 = T(camera_parameters[12*this_camera + 4]);
//			k2 = T(camera_parameters[12*this_camera + 5]);
//			p1 = T(camera_parameters[12*this_camera + 6]);
//			p2 = T(camera_parameters[12*this_camera + 7]);
//			k3 = T(camera_parameters[12*this_camera + 8]);
//			k4 = T(camera_parameters[12*this_camera + 9]);
//			k5 = T(camera_parameters[12*this_camera + 10]);
//			k6 = T(camera_parameters[12*this_camera + 11]);
//
//		} break;
//		default: {
//
//		}
//
//		}
//
//		//MatrixMultiply(K, A_hat_T, M, 4);
//		MatrixMultiply(K, A_hat_T, M, 3, 3, 3, 4);
//
//		M[12] = T(0);
//		M[13] = T(0);
//		M[14] = T(0);
//		M[15] = T(1);
////		cout << "Line 822 " << endl; //cin >> ch;
////
////		cout << "Matrix K " << endl;
////		PrintMatrix(K, 3, 3);
////
////		cout << "Matrix A hat " << endl;
////		PrintMatrix(A_hat_T, 4, 4);
////
////		cout << "Matrix M " << endl;
////		PrintMatrix(M, 4, 4);
//
//
//
//		for (int i = 0; i < number_points; i++){
//			for (int j = 0; j < 3; j++){
//				Xp[4*i + j] = T(threeDpoints[3*i + j]);
//			}
//			Xp[4*i + 3] = T(1);
//
//			//cout << "X" << endl;
//			//PrintMatrix(&Xp[4*i], 4, 1);
//
//			//cout << "Before multiply " << i << endl; //cin >> ch;
//			MatrixMultiply(A_hat_T, &Xp[4*i], &xp[3*i], 3, 4, 4, 1);
//
////			cout << "x" << endl;
////			PrintMatrix(&xp[3*i], 3, 1);
//
//
//			//cout << "After multiply " << i << endl; cin >> ch;
//
//			xp[3*i] = xp[3*i]/xp[3*i + 2];
//			xp[3*i + 1] = xp[3*i + 1]/xp[3*i + 2];
//
//			r_sqr[i] = xp[3*i]*xp[3*i] + xp[3*i + 1]*xp[3*i + 1];
//
//			xpp[3*i] = xp[3*i]* (T(1) + k1*r_sqr[i] + k2*r_sqr[i]*r_sqr[i] + k3*r_sqr[i]*r_sqr[i]*r_sqr[i])/(T(1) + k4*r_sqr[i] + k5*r_sqr[i]*r_sqr[i] + k6*r_sqr[i]*r_sqr[i]*r_sqr[i]) +
//					T(2)*p1*xp[3*i]*xp[3*i + 1]  + p2*(r_sqr[i] + T(2)*xp[3*i]);
//
//			xpp[3*i + 1] = xp[3*i + 1]* (T(1) + k1*r_sqr[i] + k2*r_sqr[i]*r_sqr[i] + k3*r_sqr[i]*r_sqr[i]*r_sqr[i])/(T(1) + k4*r_sqr[i] + k5*r_sqr[i]*r_sqr[i] + k6*r_sqr[i]*r_sqr[i]*r_sqr[i]) +
//								T(2)*p2*xp[3*i]*xp[3*i + 1]  + p1*(r_sqr[i] + T(2)*xp[3*i + 1]*xp[3*i + 1]);
//
//
//			residuals[2*i] = w*(T(twoDpoints[2*i]) - (xp[3*i]*K[0] + K[2]));
//			residuals[2*i + 1] = w*(T(twoDpoints[2*i + 1]) - (xp[3*i + 1]*K[4] + K[5]));
//			cout << "residuals " << residuals[2*i] << " " << residuals[2*i + 1] << endl;
//		}
//		//cin >> ch;
//
//		delete [] Xp;
//		delete [] xp;
//		delete [] xpp;
//		delete [] r_sqr;
//
//		return true;
//	}
//
//	//	static ceres::CostFunction* Create(double* A,
//	//			double* B, PARAM_TYPE param_type, COST_TYPE cost_type) {
//	//		return (new ceres::AutoDiffCostFunction<CF1_2, 12, 7, 7>(
//	//				new CF1_2(&A[0], &B[0], param_type, cost_type)));
//	//	}
//
//	static ceres::CostFunction* Create(double* camera_parameters, double* B, double* twoDpoints, double* threeDpoints,
//			PARAM_TYPE param_type, COST_TYPE cost_type, int number_points, int number_cameras, int this_camera, double* weighting) {
//
//
//		ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 1>* cost_function =
//				new ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 1>(
//						new RP1_2_multi(camera_parameters, B, twoDpoints, threeDpoints, param_type, cost_type, number_points, number_cameras, this_camera, weighting));
//
//
//		for (int i = 0; i <19*(number_cameras) + 7; i++){
//			cost_function->AddParameterBlock(1);
//		}
//		cost_function->SetNumResiduals(2*number_points);
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

// create is for one point only ....
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

		for (int i = 0; i < 7; i++){
			Z[i] = T(*parameters[number_cameras*19 + i]);
			X[i] = T(*parameters[this_camera*19 + i]);
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
			// don't use the parameter version for camera cali
			K[0] = T(camera_parameters[12*this_camera]);
			K[2] = T(camera_parameters[12*this_camera + 1]);
			K[4] = T(camera_parameters[12*this_camera + 2]);
			K[5] = T(camera_parameters[12*this_camera + 3]);
			k1 = T(camera_parameters[12*this_camera + 4]);
			k2 = T(camera_parameters[12*this_camera + 5]);
			p1 = T(camera_parameters[12*this_camera + 6]);
			p2 = T(camera_parameters[12*this_camera + 7]);
			k3 = T(camera_parameters[12*this_camera + 8]);
			k4 = T(camera_parameters[12*this_camera + 9]);
			k5 = T(camera_parameters[12*this_camera + 10]);
			k6 = T(camera_parameters[12*this_camera + 11]);
		} break;
		case rp2:{
			K[0] = *parameters[this_camera*19 + 7];
			K[2] = *parameters[this_camera*19 + 8];
			K[4] = *parameters[this_camera*19 + 9];
			K[5] = *parameters[this_camera*19 + 10];
			k1 = *parameters[this_camera*19 + 11];
			k2 = *parameters[this_camera*19 + 12];
			p1 = *parameters[this_camera*19 + 13];
			p2 = *parameters[this_camera*19 + 14];
			k3 = *parameters[this_camera*19 + 15];
			k4 = *parameters[this_camera*19 + 16];
			k5 = *parameters[this_camera*19 + 17];
			k6 = *parameters[this_camera*19 + 18];
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


		ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 4>* cost_function =
				new ceres::DynamicAutoDiffCostFunction<RP1_2_multi, 4>(
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

void CF1_2_one_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, double* z, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void CF1_2_multi_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void RP1_2_multi_camera(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, double* camera_params, double* x,
		std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type);

void CopyFromCalibration(vector<CaliObjectOpenCV2>& COs, double* camera_params);

void CopyToCalibration(vector<CaliObjectOpenCV2>& COs, double* camera_parameters);

#endif /* COSTFUNCTIONS1_2_HPP_ */
