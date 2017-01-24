/*
 * ComparisonMethods.hpp
 *
 *  Created on: Nov 30, 2015
 *      Author: atabb
 */

#ifndef COMPARISONMETHODS_HPP_
#define COMPARISONMETHODS_HPP_

#include "CostFunctions.hpp"

template <typename T>
void MatrixTranspose(const T* X, T* M){

	M[0] = X[0];
	M[1] = X[3];
	M[2] = X[6];

	M[3] = X[1];
	M[4] = X[4];
	M[5] = X[7];

	M[6] = X[2];
	M[7] = X[5];
	M[8] = X[8];
}

template <typename T>
void ReadFromFullToRotationMatrix(const T* X, T* M){

	M[0] = X[0];
	M[1] = X[1];
	M[2] = X[2];

	M[3] = X[4];
	M[4] = X[5];
	M[5] = X[6];

	M[6] = X[8];
	M[7] = X[9];
	M[8] = X[10];

	//	cout << "rot matrix " << endl;
	//	for (int i = 0; i <9; i++){
	//		cout << M[i] << " ";
	//	}
	//	cout << endl;
	//	char ch; cin >> ch;

}

template <typename T>
void ReadFromParameterMatrixToFullMatrix(const T* X, T* M){

	for (int i = 0; i < 12; i++){
		M[i] = X[i];
	}

	M[12] = T(0);
	M[13] = T(0);
	M[14] = T(0);
	M[15] = T(1);


	//	cout << "rot matrix " << endl;
	//	for (int i = 0; i <9; i++){
	//		cout << M[i] << " ";
	//	}
	//	cout << endl;
	//	char ch; cin >> ch;

}

template <typename T>
void IdentityMatrix(T* M){

	M[0] = T(1);
	M[1] = T(0);
	M[2] = T(0);

	M[3] = T(0);
	M[4] = T(1);
	M[5] = T(0);

	M[6] = T(0);
	M[7] = T(0);
	M[8] = T(1);

}



//struct DH {
//	DH(double* A, double* B, double mu1, double mu3, double mu4): A(A), B(B), mu1(mu1),	mu3(mu3), mu4(mu4){
//	}
//
//	template <typename T>
//	bool operator()(const T* const X,
//			const T* const Z,
//			T* residuals) const {
//		// X and Z are 6 parameters each, and our unknowns
//		// need to represent these each as matrices
//
//		//The {pitch,roll,yaw} Euler angles are rotations around the {x,y,z} axes, respectively.
//		//They are applied in that same order, so the total rotation R is Rz * Ry * Rx.
//
//		//		T XM[16];
//		//		T ZM[16];
//		//
//		//		// X and Z are already in matrix form ....
//		//
//		//		switch (param_type){
//		//		case Euler: {
//		//			Convert6ParameterEulerAngleRepresentationIntoMatrix(X, XM);
//		//			Convert6ParameterEulerAngleRepresentationIntoMatrix(Z, ZM);
//		//		}	break;
//		//		case AxisAngle: {
//		//			Convert6ParameterAxisAngleRepresentationIntoMatrix(X, XM);
//		//			Convert6ParameterAxisAngleRepresentationIntoMatrix(Z, ZM);
//		//		}	break;
//		//		case Quaternion: {
//		//			Convert7ParameterQuaternionRepresentationIntoMatrix(X, XM);
//		//			Convert7ParameterQuaternionRepresentationIntoMatrix(Z, ZM);
//		//		}	break;
//		//		}
//
//		T term0[16];
//		T term1[16];
//		T term2[9];
//		T term3[9];
//		T RX[9];
//		T RZ[9];
//
//		T XM[16];
//		T ZM[16];
//
//		ReadFromFullToRotationMatrix(X, RX);
//		ReadFromFullToRotationMatrix(Z, RZ);
//
//		ReadFromParameterMatrixToFullMatrix(X, XM);
//		ReadFromParameterMatrixToFullMatrix(Z, ZM);
//
//		T I[9];
//		IdentityMatrix(I);
//
//		T RXT[9];
//		T RZT[9];
//
//		MatrixTranspose(RX, RXT);
//		MatrixTranspose(RZ, RZT);
//
//		// ||AX-ZB||
//		// have to convert to T
//		T AT[16];
//		T BT[16];
//		for (int i = 0; i < 16; i++){
//			AT[i] = T(A[i]);
//			BT[i] = T(B[i]);
//		}
//
//		MatrixMultiply(AT, XM, term0, 4);
//		MatrixMultiply(ZM, BT, term1, 4);
//
//		MatrixMultiply(RX, RXT, term2, 3);
//		MatrixMultiply(RZ, RZT, term3, 3);
//
//
//		for (int i = 0; i < 12; i++){
//			residuals[i] = T(mu1)*(term0[i] - term1[i]);
//
//		}
//
//
//
//		for (int i = 0; i < 9; i++){
//			residuals[i + 12] = T(mu4)*(term2[i] - I[i]);
//		}
//
//
//		for (int i = 0; i < 9; i++){
//			residuals[i + 21] = T(mu4)*(term3[i] - I[i]);
//		}
//
//
//		return true;
//	}
//
//	static ceres::CostFunction* Create(double* A,
//			double* B, double mu1, double mu3, double mu4) {
//		return (new ceres::AutoDiffCostFunction<DH, 30, 12, 12>(
//				new DH(&A[0], &B[0], mu1, mu3, mu4)));
//	}
//
//	double* A;
//	double* B;
//	double mu1;
//	double mu3;
//	double mu4;
//};



// Helpers
void ConvertFromMatrixToDouble(double* D, Matrix& M, int rows, int cols);
void ConvertFromDoubleToMatrix(double* D, Matrix& M, int rows, int cols);


#endif /* COMPARISONMETHODS_HPP_ */
