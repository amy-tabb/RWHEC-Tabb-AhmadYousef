/*
 * CostFunctions1_2.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: atabb
 */
#include "CostFunctions1_2.hpp"


void CF1_2_one_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, double* z, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){

	Problem problem;

	int number_images = As[0].size();
	double* A = new double[16*number_images];
	double* B = new double[16*number_images];

	// change all of this newmat stuff out ...
	for (int i = 0; i < int(As[0].size()); i++) {
		//	for (int i = 0; i < 1; i++) {

		for (int r = 0, in = 0; r < 4; r++){
			for (int c = 0; c < 4; c++, in++){
				A[in + 16*i] = As[0][i](r + 1, c + 1);
				B[in + 16*i] = Bs[i](r + 1, c + 1);
			}
		}


		ceres::CostFunction* cost_function =
				CF1_2::Create(
						&A[16*i], &B[16*i], param_type, cost_type);
		problem.AddResidualBlock(cost_function,
				NULL /* squared loss */,
				x,
				z);
	}


	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	out << summary.FullReport() << endl;
	std::cout << summary.BriefReport() << "\n";

	out << "X ";
	for (int i = 0; i < 6; i++){
		out << x[i] << " ";
	}
	out << endl;

	out << "Z ";
	for (int i = 0; i < 6; i++){
		out << z[i] << " ";
	}
	out << endl;


	delete [] A;
	delete [] B;
}
//void CF1_2_multi_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){
//
//	Problem problem;
//
//	int number_cameras = As.size();
//	int number_images = As[0].size();
//	double* A = new double[16*number_images* number_cameras];
//	double* B = new double[16*number_images];
//
//	vector<double*> parameter_blocks;
//
//	for (int i = 0; i < 7*(number_cameras + 1); i++){
//		parameter_blocks.push_back(&x[i]);
//	}
//
//	//	for (int i = 0; i < 7; i++){
//	//		parameter_blocks.push_back(&z[i]);
//	//	}
//
//	out << "Preliminary" << endl;
//	out << "X  ... Z" << endl;
//	for (int j = 0; j < number_cameras + 1; j++){
//		for (int i = 0; i < 7; i++){
//			out << x[7*j + i] << " ";
//		}
//		out << endl;
//	}
//
//
//	//	out << "Z ";
//	//	for (int i = 0; i < 7; i++){
//	//		out << x[7*(number_cameras + 1) + i] << " ";
//	//	}
//	out << endl;
//
//	// change all of this newmat stuff out ...
//	for (int i = 0; i < number_images; i++) {
//		//	for (int i = 0; i < 1; i++) {
//
//		// for each image, B stays the same
//		for (int r = 0, in = 0; r < 4; r++){
//			for (int c = 0; c < 4; c++, in++){
//				B[in + 16*i] = Bs[i](r + 1, c + 1);
//			}
//		}
//
//		for (int j = 0; j < number_cameras; j++){
//
//			for (int r = 0, in = 0; r < 4; r++){
//				for (int c = 0; c < 4; c++, in++){
//					A[16*i*number_cameras + j*16 + in] = As[0][i](r + 1, c + 1);
//
//				}
//			}
//		}
//	// probelm is the indexing ...
//		ceres::CostFunction* cost_function =
//				CF1_2_general::Create(
//						&A[16*i*number_cameras], &B[16*i], param_type, cost_type, number_cameras);
//		problem.AddResidualBlock(cost_function,
//				NULL /* squared loss */,
//				parameter_blocks);
//	}
//
//
//	// Run the solver!
//	Solver::Options options;
//	options.linear_solver_type = ceres::DENSE_QR;
//	options.minimizer_progress_to_stdout = true;
//	Solver::Summary summary;
//	Solve(options, &problem, &summary);
//
//	out << summary.FullReport() << endl;
//	std::cout << summary.BriefReport() << "\n";
//
//	out << "X  ... Z" << endl;
//	for (int j = 0; j < number_cameras + 1; j++){
//		for (int i = 0; i < 7; i++){
//			out << x[7*j + i] << " ";
//		}
//		out << endl;
//	};
//
//
//	delete [] A;
//	delete [] B;
//}

//void CF1_2_multi_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){
//
//	Problem problem;
//
//	int number_cameras = As.size();
//	int number_images = As[0].size();
//	double* A = new double[16*number_images];
//	double* B = new double[16*number_images*number_cameras];
//
//	vector<double*> parameter_blocks;
//
//	for (int i = 0; i < 7*(number_cameras + 1); i++){
//		parameter_blocks.push_back(&x[i]);
//	}
//
//	//	for (int i = 0; i < 7; i++){
//	//		parameter_blocks.push_back(&z[i]);
//	//	}
//
//	out << "Preliminary" << endl;
//	out << "X  ... Z" << endl;
//	for (int j = 0; j < number_cameras + 1; j++){
//		for (int i = 0; i < 7; i++){
//			out << x[7*j + i] << " ";
//		}
//		out << endl;
//	}
//
//	char ch;
//
//	//	out << "Z ";
//	//	for (int i = 0; i < 7; i++){
//	//		out << x[7*(number_cameras + 1) + i] << " ";
//	//	}
//	out << endl;
//
//	// change all of this newmat stuff out ...
//	for (int i = 0; i < number_images; i++) {
//		//	for (int i = 0; i < 1; i++) {
//
//		// for each image, B stays the same
//		for (int r = 0, in = 0; r < 4; r++){
//			for (int c = 0; c < 4; c++, in++){
//				B[in + 16*i] = Bs[i](r + 1, c + 1);
//			}
//		}
//
//		for (int j = 0; j < number_cameras; j++){
//
//			cout << "Filling in ";
//			for (int r = 0, in = 0; r < 4; r++){
//				for (int c = 0; c < 4; c++, in++){
//					A[16*i*number_cameras + j*16 + in] = As[0][i](r + 1, c + 1);
//					cout << 16*i*number_cameras + j*16 + in << " ";
//				}
//			}
//			cout << endl;
//			cout << "beginning index to cost function " << 16*i*number_cameras + j*16 << endl << endl;
//			//cin >> ch;
//
//			ceres::CostFunction* cost_function =
//					CF1_2_generalB::Create(
//							&A[16*i*number_cameras + j*16], &B[16*i], param_type, cost_type, number_cameras, j);
//			problem.AddResidualBlock(cost_function,
//					NULL /* squared loss */,
//					parameter_blocks);
//		}
//	}
//
//
//	// Run the solver!
//	Solver::Options options;
//	options.linear_solver_type = ceres::DENSE_QR;
//	options.minimizer_progress_to_stdout = true;
//	Solver::Summary summary;
//	Solve(options, &problem, &summary);
//
//	out << summary.FullReport() << endl;
//	std::cout << summary.BriefReport() << "\n";
//
//	out << "X  ... Z" << endl;
//	for (int j = 0; j < number_cameras + 1; j++){
//		for (int i = 0; i < 7; i++){
//			out << x[7*j + i] << " ";
//		}
//		out << endl;
//	};
//
//	cout << "After solving " << endl;
//	cin >> ch;
//
//	parameter_blocks.clear();
//	delete [] A;
//	delete [] B;
//}

void CF1_2_multi_camera(vector< vector<Matrix> >& As, vector<Matrix>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){

	// make flexible for different numbers of images for different cameras ....
	Problem problem;

	int number_cameras = As.size();
	// all of them will have the same size, null matrices will be in places where the the camera does not view the pattern.
	int number_images = As[0].size();

	vector<double*> AA;
	//double* A = new double[16*number_images];
	for (int i = 0; i < number_cameras; i++){
		AA.push_back(new double[16*number_images]);
	}
	double* B = new double[16*number_images];

	vector<double*> parameter_blocks;

	int minimum_number = number_images;

	vector<int> number_of_images_per_camera;
	vector<double> weighting;

	// count number for each camera ....
	for (int j = 0; j < number_cameras; j++){
		int count = 0;

		for (int i = 0; i < number_images; i++){
			if (As[j][i].nrows() > 0){
				count++;
			}
		}

		number_of_images_per_camera.push_back(count);

		if (count < minimum_number){
			minimum_number = count;
		}
	}

	out << "Camera number, number of images per camera, weighting " << endl;
	for (int j = 0; j < number_cameras; j++){
		weighting.push_back(double(minimum_number)/double(number_of_images_per_camera[j]));
		out << j << " " << number_of_images_per_camera[j] << " " << weighting[j] << endl;
	}
	out << endl;




	for (int i = 0; i < 7*(number_cameras + 1); i++){
		parameter_blocks.push_back(&x[i]);
	}

	out << "Preliminary" << endl;
	out << "X  ... Z" << endl;
	for (int j = 0; j < number_cameras + 1; j++){
		for (int i = 0; i < 7; i++){
			out << x[7*j + i] << " ";
		}
		out << endl;
	}

	char ch;
	//cin >> ch;

	//	out << "Z ";
	//	for (int i = 0; i < 7; i++){
	//		out << x[7*(number_cameras + 1) + i] << " ";
	//	}
	out << endl;

	for (int j = 0; j < number_cameras; j++){

		// change all of this newmat stuff out ...
		for (int i = 0; i < number_images; i++) {
			//	for (int i = 0; i < 1; i++) {


			if (As[j][i].size() > 0){
				// for each image, B stays the same
				for (int r = 0, in = 0; r < 4; r++){
					for (int c = 0; c < 4; c++, in++){
						B[in + 16*i] = Bs[i](r + 1, c + 1);
					}
				}


				for (int r = 0, in = 0; r < 4; r++){
					for (int c = 0; c < 4; c++, in++){
						AA[j][16*i + in] = As[j][i](r + 1, c + 1);
						//cout << 16*i + in << " ";
					}
				}


				ceres::CostFunction* cost_function =
						CF1_2_multi::Create(
								&AA[j][16*i], &B[16*i], param_type, cost_type, number_cameras, j, &weighting[j]);
				problem.AddResidualBlock(cost_function,
						NULL /* squared loss */,
						parameter_blocks);
			}
		}
	}


	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	out << summary.FullReport() << endl;
	std::cout << summary.BriefReport() << "\n";

	out << "X  ... Z" << endl;
	for (int j = 0; j < number_cameras + 1; j++){
		for (int i = 0; i < 7; i++){
			out << x[7*j + i] << " ";
		}
		out << endl;
	};

	cout << "After solving " << endl;
	//cin >> ch;

	parameter_blocks.clear();
	//delete [] A;
	for (int i = 0; i < number_cameras; i++){
		delete [] AA[i];
	}
	delete [] B;
}

//void LevMarMethodReprojectionError3MultCams(vector< vector<Matrix> >& As, vector<Matrix>& Bs, vector<Matrix>& Xs, Matrix& Z,
//		vector<CaliObjectOpenCV2>& COs, std::ofstream& out){
//
//	// Here, we let X by X inverse ... so we need to reprocess before leaving ....
//	char ch;
//
//	CaliData C;
//	C.As = 0;
//	C.Amults = &As;
//	C.Bs = &Bs;
//	C.CO = 0;
//	C.COs = &COs;
//
//	ColumnVector world_c(3);
//	for (int h=0; h< COs[0].chess_h; h++) {
//		for (int w=0; w< COs[0].chess_w; w++) {
//			world_c(1) = (double)(w * COs[0].mm_width);
//			world_c(2) = (double)(h * COs[0].mm_height);
//			world_c(3) = 0;
//
//			C.world_coordinates.push_back(world_c);
//		}
//	}
//
//	int number_cameras = Xs.size();
//	int number_per_camera_params = 5 + 8 + 6;
//	// each image has world_points.size() items ....
//	int n = number_cameras*As[0].size()*C.world_coordinates.size()*2;
//	int m = number_cameras*number_per_camera_params + 6;
//	// 3 angles each, then 3 translation compoenents each + 5 components for K (angles X, trans X, andlges Z, trans Z,
//	//K parameters last.).
//	// order is internal parameters, distortion coeffs, Xs (rotation and translation) for all cams ... then Zs
//
//	double x[n];
//
//
//
//
//	//double jac[n*m];
//
//	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
//
//	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
//	opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing
//
//	// fill in using CO and undistort ....
//	//int counter = 0;
//	//cvmSet(corners2d, cnt, 0, (double)all_corners[cnt].x);   // image u
//	//cvmSet(corners2d, cnt, 1, (double)all_corners[cnt].y);   // image v
//
//
//	//	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
//	//	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
//	//
//	//	for (int i = 0; i < 3; i++){
//	//		for (int j = 0; j < 3; j++){
//	//			cameraMatrix.at<double>(i, j)  = COs->A[i][j];
//	//		}
//	//	}
//	//
//	//	for (int i = 0; i < 8; i++){
//	//		distCoeffs.at<double>(i, 0) = CO->k[i];
//	//	}
//	//
//	//	cout << "Cali matrix " << endl << cameraMatrix << endl;
//	//	cout << "coefficients " << distCoeffs << endl;
//
//
//	cv::Mat view, rview, map1, map2;
//	//cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//	//cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, CO->image_size, 1, CO->image_size, 0),
//	//			CO->image_size, CV_16SC2, map1, map2);
//	//	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//	//		cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, CO->image_size, 1, CO->image_size, 0),
//	//					CO->image_size, CV_32FC1, map1, map2);
//
//	// now, x are the corrected image values .....
//	for (int i = 0; i < n; i++){
//		x[i] = 0;
//	}
//
//	int counter = 0;
//	cv::Point2f point;
//
//	for (int cam = 0; cam < number_cameras; cam++){
//		//C.image_coordinates_x_y_mult.push_back(vector<double>());
//
//		for (int i = 0; i < COs[cam].Rts.size(); i++){
//			//	for (int i = 0; i < 1; i++){
//			for (int j = 0; j < COs[cam].chess_h * COs[cam].chess_w; j++){
//
//				// don't worry about distortion for
//				point = COs[cam].all_points[COs[cam].number_internal_images_written + i][j];
//
//				C.image_coordinates_x_y.push_back(point.x);
//				C.image_coordinates_x_y.push_back(point.y);
//
//				//			x[counter] = point.x;
//				//			counter++;
//				//			x[counter]= point.y;
//				//			counter++;
//
//				//C.image_coordinates_x_y.push_back(map1.at<float>(point.y, point.x));
//				//C.image_coordinates_x_y.push_back(map2.at<float>(point.y, point.x));
//				//			x[counter] = map1.at<float>(point.y, point.x);
//				//			counter++;
//				//
//				//			x[counter] = map2.at<float>(point.y, point.x);
//				//			counter++;
//				//
//				//			if (counter > n){
//				//				cout << "Counter exceeds n " << counter << ", " << n << endl;
//				//				exit(1);
//				//			}
//				//
//				//cout << "Original point " << point.x << ", " << point.y << endl;
//				//cout << "Corrected      " << map1.at<float>(point.y, point.x) << ", " << map2.at<float>(point.y, point.x) << endl;
//			}
//
//		}
//	}
//
//	//cv:Size map_size = map1.size();
//	cout << "Size of map is " << map1.rows << ", " << map1.cols << endl;
//
//	cout << "n is " << n << endl;
//	cout << "Number of image points is " << C.image_coordinates_x_y.size() << endl;
//	//cout << "Finished with n " << endl; cin >> ch;
//
//
//
//
//	// convert X, Z, into parameter vector -- LATER
//
//	double parameter_vector[n];
//	for (int i = 0; i < m; i++){
//		parameter_vector[i] = 0;
//	}
//
//	vector<double> anglesX(3, 0);
//
//	for (int cam = 0; cam < number_cameras; cam++){
//		parameter_vector[cam*number_per_camera_params + 0] = COs[cam].A[0][0];
//		parameter_vector[cam*number_per_camera_params + 1] = COs[cam].A[0][1];
//		parameter_vector[cam*number_per_camera_params + 2] = COs[cam].A[0][2];
//		parameter_vector[cam*number_per_camera_params + 3] = COs[cam].A[1][1];
//		parameter_vector[cam*number_per_camera_params + 4] = COs[cam].A[1][2];
//
//		parameter_vector[cam*number_per_camera_params + 5] = COs[cam].k[0]; // dist1
//		parameter_vector[cam*number_per_camera_params + 6] = COs[cam].k[1]; // dist2
//		parameter_vector[cam*number_per_camera_params + 7] = COs[cam].k[2]; //p1
//		parameter_vector[cam*number_per_camera_params + 8] = COs[cam].k[3]; //p2
//		parameter_vector[cam*number_per_camera_params + 9] = COs[cam].k[4]; //dist3
//		parameter_vector[cam*number_per_camera_params + 10] = COs[cam].k[5]; //dist4
//		parameter_vector[cam*number_per_camera_params + 11] = COs[cam].k[6]; //dist5
//		parameter_vector[cam*number_per_camera_params + 12] = COs[cam].k[7]; //dist6
//
//
//		Matrix Xi = Xs[cam].i();
//		ExtractAnglesAndTransformAgain(Xi, anglesX);
//
//		parameter_vector[cam*number_per_camera_params + 13] = anglesX[0];
//		parameter_vector[cam*number_per_camera_params + 14] = anglesX[1];
//		parameter_vector[cam*number_per_camera_params + 15] = anglesX[2];
//
//		parameter_vector[cam*number_per_camera_params + 16] = Xi(1, 4);
//		parameter_vector[cam*number_per_camera_params + 17] = Xi(2, 4);
//		parameter_vector[cam*number_per_camera_params + 18] = Xi(3, 4);
//	}
//
//	ExtractAnglesAndTransformAgain(Z, anglesX);
//
//	parameter_vector[number_cameras*number_per_camera_params + 0] = anglesX[0];
//	parameter_vector[number_cameras*number_per_camera_params + 1] = anglesX[1];
//	parameter_vector[number_cameras*number_per_camera_params + 2] = anglesX[2];
//
//	parameter_vector[number_cameras*number_per_camera_params + 3] = Z(1, 4);
//	parameter_vector[number_cameras*number_per_camera_params + 4] = Z(2, 4);
//	parameter_vector[number_cameras*number_per_camera_params + 5] = Z(3, 4);
//
//
//	//	parameter_vector[12] = CO->A[0][0];
//	//	parameter_vector[13] = CO->A[0][1];
//	//	parameter_vector[14] = CO->A[0][2];
//	//	parameter_vector[15] = CO->A[1][1];
//	//	parameter_vector[16] = CO->A[1][2];
//
//	out << endl;
//	out << "Intial solution ";
//	for(int i=0; i<m; ++i){
//		out <<	parameter_vector[i] << " ";
//	}
//	out << endl << endl;
//
//	int ret;
//
//	//residuals_reprojection2_mults(parameter_vector, x, m, n, &C);
//	//cout << "e " << e << endl;
//	//residuals_reprojection3_mults(parameter_vector, x, m, n, &C);
//	//ret = dlevmar_dif(residuals_reprojection3_mults, parameter_vector, x, m, n, 1000, opts, info, NULL, NULL, &C); // with analytic Jacobian
//
//	//ret = dlevmar_dif(residuals_reprojection3_mults, parameter_vector, x, m, n, 1000, opts, info, NULL, NULL, &C);
//	ret = dlevmar_der(residuals_reprojection3_mults, jacobian_reprojection3_mults, parameter_vector, x, m, n, 2000, opts, info, NULL, NULL, &C); // with analytic Jacobian
//	//
//	/* O: information regarding the minimization. Set to NULL if don't care
//	 * info[0]= ||e||_2 at initial p.
//	 * info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, mu/max[J^T J]_ii ], all computed at estimated p.
//	 * info[5]= # iterations,
//	 * info[6]=reason for terminating: 1 - stopped by small gradient J^T e
//	 *                                 2 - stopped by small Dp
//	 *                                 3 - stopped by itmax
//	 *                                 4 - singular matrix. Restart from current p with increased mu
//	 *                                 5 - no further error reduction is possible. Restart with increased mu
//	 *                                 6 - stopped by small ||e||_2
//	 *                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values. This is a user error
//	 * info[7]= # function evaluations
//	 * info[8]= # Jacobian evaluations
//	 * info[9]= # linear systems solved, i.e. # attempts for reducing error
//	 */
//
//
//	//printf("Results for %s:\n", probname[problem]);
//	printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);
//	for(int i=0; i<m; ++i)
//		printf("%.7g ", parameter_vector[i]);
//	printf("\n\nMinimization info:\n");
//	for(int i=0; i<LM_INFO_SZ; ++i)
//		printf("%g ", info[i]);
//	printf("\n");
//
//	out << "Levenberg-Marquardt returned " << ret << " in " << info[5] << " iter, reason " << info[6] << endl;
//	out << " Solution ";
//	for(int i=0; i<m; ++i){
//		out <<	parameter_vector[i] << " ";
//	}
//	out << endl << endl;
//
//	out << "minimization information " << endl;
//	for(int i=0; i<LM_INFO_SZ; ++i){
//		out << info[i] << " ";
//	}
//	out << endl << endl;
//
//
//
//
//
//	//R = Rx*Ry*Rz;
//	// X params first, then Y params.
//	Matrix Rx(3, 3);
//	Matrix Ry(3, 3);
//	Matrix Rz(3, 3);
//
//	for (int cam = 0; cam < number_cameras; cam++){
//		COs[cam].A[0][0] = parameter_vector[cam*number_per_camera_params + 0];
//		COs[cam].A[0][1] = parameter_vector[cam*number_per_camera_params + 1];
//		COs[cam].A[0][2] = parameter_vector[cam*number_per_camera_params + 2];
//		COs[cam].A[1][1] = parameter_vector[cam*number_per_camera_params + 3];
//		COs[cam].A[1][2] = parameter_vector[cam*number_per_camera_params + 4];
//
//		COs[cam].k[0] = parameter_vector[cam*number_per_camera_params + 5]; // dist1
//		COs[cam].k[1] = parameter_vector[cam*number_per_camera_params + 6]; // dist2
//		COs[cam].k[2] = parameter_vector[cam*number_per_camera_params + 7]; //p1
//		COs[cam].k[3] = parameter_vector[cam*number_per_camera_params + 8]; //p2
//		COs[cam].k[4] = parameter_vector[cam*number_per_camera_params + 9]; //dist3
//		COs[cam].k[5] = parameter_vector[cam*number_per_camera_params + 10]; //dist4
//		COs[cam].k[6] = parameter_vector[cam*number_per_camera_params + 11]; //dist5
//		COs[cam].k[7] = parameter_vector[cam*number_per_camera_params + 12]; //dist6
//
//		Rx.Row(1) << 1 <<  0 << 0;
//		Rx.Row(2) << 0 << cos(parameter_vector[cam*number_per_camera_params + 13]) << -sin(parameter_vector[cam*number_per_camera_params + 13]);
//		Rx.Row(3) << 0 << sin(parameter_vector[cam*number_per_camera_params + 13]) << cos(parameter_vector[cam*number_per_camera_params + 13]);
//
//		Ry.Row(1) << cos(parameter_vector[cam*number_per_camera_params + 14]) << 0 << sin(parameter_vector[cam*number_per_camera_params + 14]);
//		Ry.Row(2) << 0 << 1 << 0;
//		Ry.Row(3) << -sin(parameter_vector[cam*number_per_camera_params + 14]) << 0 << cos(parameter_vector[cam*number_per_camera_params + 14]);
//
//		Rz.Row(1) << cos(parameter_vector[cam*number_per_camera_params + 15]) << -sin(parameter_vector[cam*number_per_camera_params + 15]) << 0;
//		Rz.Row(2) << sin(parameter_vector[cam*number_per_camera_params + 15]) << cos(parameter_vector[cam*number_per_camera_params + 15]) << 0;
//		Rz.Row(3) << 0 << 0 << 1;
//
//
//		// X is inverted ....
//		Xs[cam].SubMatrix(1, 3, 1, 3) = (Rx*Ry*Rz);
//		Xs[cam](1, 4) = parameter_vector[cam*number_per_camera_params + 16];
//		Xs[cam](2, 4) = parameter_vector[cam*number_per_camera_params + 17];
//		Xs[cam](3, 4) = parameter_vector[cam*number_per_camera_params + 18];
//
//		Xs[cam] = Xs[cam].i();
//	}
//
//
//
//
//	Rx.Row(1) << 1 <<  0 << 0;
//	Rx.Row(2) << 0 << cos(parameter_vector[number_cameras*number_per_camera_params + 0]) << -sin(parameter_vector[number_cameras*number_per_camera_params + 0]);
//	Rx.Row(3) << 0 << sin(parameter_vector[number_cameras*number_per_camera_params + 0]) << cos(parameter_vector[number_cameras*number_per_camera_params + 0]);
//
//	Ry.Row(1) << cos(parameter_vector[number_cameras*number_per_camera_params + 1]) << 0 << sin(parameter_vector[number_cameras*number_per_camera_params + 1]);
//	Ry.Row(2) << 0 << 1 << 0;
//	Ry.Row(3) << -sin(parameter_vector[number_cameras*number_per_camera_params + 1]) << 0 << cos(parameter_vector[number_cameras*number_per_camera_params + 1]);
//
//	Rz.Row(1) << cos(parameter_vector[number_cameras*number_per_camera_params + 2]) << -sin(parameter_vector[number_cameras*number_per_camera_params + 2]) << 0;
//	Rz.Row(2) << sin(parameter_vector[number_cameras*number_per_camera_params + 2]) << cos(parameter_vector[number_cameras*number_per_camera_params + 2]) << 0;
//	Rz.Row(3) << 0 << 0 << 1;
//
//
//	Z.SubMatrix(1, 3, 1, 3) = Rx*Ry*Rz;
//	Z(1, 4) = parameter_vector[number_cameras*number_per_camera_params + 3];
//	Z(2, 4) = parameter_vector[number_cameras*number_per_camera_params + 4];
//	Z(3, 4) = parameter_vector[number_cameras*number_per_camera_params + 5];
//
//}


//vector<CaliObjectOpenCV2>& COs, std::ofstream& out){
//
//	// Here, we let X by X inverse ... so we need to reprocess before leaving ....
//	char ch;
//
//	CaliData C;
//	C.As = 0;
//	C.Amults = &As;
//	C.Bs = &Bs;
//	C.CO = 0;
//	C.COs = &COs;
//
//	ColumnVector world_c(3);
//	for (int h=0; h< COs[0].chess_h; h++) {
//		for (int w=0; w< COs[0].chess_w; w++) {
//			world_c(1) = (double)(w * COs[0].mm_width);
//			world_c(2) = (double)(h * COs[0].mm_height);
//			world_c(3) = 0;
//
//			C.world_coordinates.push_back(world_c);
//		}
//	}
//
//	int number_cameras = Xs.size();
//	int number_per_camera_params = 5 + 8 + 6;
//	// each image has world_points.size() items ....
//	int n = number_cameras*As[0].size()*C.world_coordinates.size()*2;
//	int m = number_cameras*number_per_camera_params + 6;
//	// 3 angles each, then 3 translation compoenents each + 5 components for K (angles X, trans X, andlges Z, trans Z,
//	//K parameters last.).
//	// order is internal parameters, distortion coeffs, Xs (rotation and translation) for all cams ... then Zs
//
//	double x[n];
//
//
//
//
//	//double jac[n*m];
//
//	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
//
//	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
//	opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing
//
//	// fill in using CO and undistort ....
//	//int counter = 0;
//	//cvmSet(corners2d, cnt, 0, (double)all_corners[cnt].x);   // image u
//	//cvmSet(corners2d, cnt, 1, (double)all_corners[cnt].y);   // image v
//
//
//	//	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
//	//	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
//	//
//	//	for (int i = 0; i < 3; i++){
//	//		for (int j = 0; j < 3; j++){
//	//			cameraMatrix.at<double>(i, j)  = COs->A[i][j];
//	//		}
//	//	}
//	//
//	//	for (int i = 0; i < 8; i++){
//	//		distCoeffs.at<double>(i, 0) = CO->k[i];
//	//	}
//	//
//	//	cout << "Cali matrix " << endl << cameraMatrix << endl;
//	//	cout << "coefficients " << distCoeffs << endl;
//
//
//	cv::Mat view, rview, map1, map2;
//	//cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//	//cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, CO->image_size, 1, CO->image_size, 0),
//	//			CO->image_size, CV_16SC2, map1, map2);
//	//	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
//	//		cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, CO->image_size, 1, CO->image_size, 0),
//	//					CO->image_size, CV_32FC1, map1, map2);
//
//	// now, x are the corrected image values .....
//	for (int i = 0; i < n; i++){
//		x[i] = 0;
//	}
//
//	int counter = 0;
//	cv::Point2f point;
//
//	for (int cam = 0; cam < number_cameras; cam++){
//		//C.image_coordinates_x_y_mult.push_back(vector<double>());
//
//		for (int i = 0; i < COs[cam].Rts.size(); i++){
//			//	for (int i = 0; i < 1; i++){
//			for (int j = 0; j < COs[cam].chess_h * COs[cam].chess_w; j++){
//
//				// don't worry about distortion for
//				point = COs[cam].all_points[COs[cam].number_internal_images_written + i][j];
//
//				C.image_coordinates_x_y.push_back(point.x);
//				C.image_coordinates_x_y.push_back(point.y);
//
//				//			x[counter] = point.x;
//				//			counter++;
//				//			x[counter]= point.y;
//				//			counter++;
//
//				//C.image_coordinates_x_y.push_back(map1.at<float>(point.y, point.x));
//				//C.image_coordinates_x_y.push_back(map2.at<float>(point.y, point.x));
//				//			x[counter] = map1.at<float>(point.y, point.x);
//				//			counter++;
//				//
//				//			x[counter] = map2.at<float>(point.y, point.x);
//				//			counter++;
//				//
//				//			if (counter > n){
//				//				cout << "Counter exceeds n " << counter << ", " << n << endl;
//				//				exit(1);
//				//			}
//				//
//				//cout << "Original point " << point.x << ", " << point.y << endl;
//				//cout << "Corrected      " << map1.at<float>(point.y, point.x) << ", " << map2.at<float>(point.y, point.x) << endl;
//			}
//
//		}
//	}



//for (int i = 0; i < 3; i++){
//		for (int j = 0; j < 3; j++){
//			cameraMatrix.at<double>(i, j)  = CO->A[i][j];
//		}
//	}
//
//	for (int i = 0; i < 8; i++){
//		distCoeffs.at<double>(i, 0) = CO->k[i];
//	}
//
//	string filename;
//	int non_blanks = 0;
//	//cout << "As size " << As.size() << endl;
//	for (int i = 0; i < int(As.size()); i++){
//		if (CO->all_points[CO->number_internal_images_written + i].size() > 0){
//			//cout << "Go " << i << " " << CO->all_points[CO->number_internal_images_written + i].size() <<  endl;
//			newA = Z*Bs[i]*X.i();
//
//			for (int r = 0; r < 3; r++){
//				for (int c = 0; c < 3; c++){
//					R.at<double>(r, c) = newA(r + 1, c + 1);
//				}
//				tvec.at<double>(r, 0) = newA(r + 1, 4);
//			}
//
//			cv::Rodrigues(R, rvec);
//
//			cv::projectPoints( cv::Mat(CO->all_3d_corners[CO->number_internal_images_written + non_blanks]), rvec, tvec, cameraMatrix,  // project
//					distCoeffs, imagePoints2);
//			err = cv::norm(cv::Mat(CO->all_points[CO->number_internal_images_written + i]), cv::Mat(imagePoints2), CV_L2);              // difference
//
//			reproj_error        += err*err;                                             // su
//
//			im = CO->external_images[i].clone();
//			cv::cvtColor(im, gray, CV_BGR2GRAY);
//			cv::cvtColor(gray, im, CV_GRAY2BGR);
//
//
//			for (int j = 0; j < int(imagePoints2.size()); j++){
//				cv::line(im, imagePoints2[j], CO->all_points[CO->number_internal_images_written + i][j], cv::Scalar(255, 0, 0), 2, 8);
//			}
//
//
//			filename = directory + "/reproj" + ToString<int>(cam_number) + "_" + ToString<int>(i) + ".png";
//
//			cv::imwrite(filename.c_str(), im);
//			non_blanks++;
//		}
//
//	}

void CopyFromCalibration(vector<CaliObjectOpenCV2>& COs, double* camera_params){
	int number_cameras = COs.size();
	for (int i = 0; i < number_cameras; i++){
		camera_params[12*i] = COs[i].A[0][0];
		camera_params[12*i + 1] = COs[i].A[0][2];
		camera_params[12*i + 2] = COs[i].A[1][1];
		camera_params[12*i + 3] = COs[i].A[1][2];

		for (int j = 0; j < 8; j++){
			camera_params[12*i + 4 + j] = COs[i].k[j];
		}
	}
}

void CopyToCalibration(vector<CaliObjectOpenCV2>& COs, double* camera_parameters){
	int number_cameras = COs.size();
	for (int i = 0; i < number_cameras; i++){
		COs[i].A[0][0] = camera_parameters[12*i];
		COs[i].A[0][2] = camera_parameters[12*i + 1];
		COs[i].A[1][1] = camera_parameters[12*i + 2];
		COs[i].A[1][2] = camera_parameters[12*i + 3];

		for (int j = 0; j < 8; j++){
			COs[i].k[j] = camera_parameters[12*i + 4 + j];
		}
	}
}
void RP1_2_multi_camera(vector<CaliObjectOpenCV2>& COs, vector<Matrix>& Bs, double* camera_params, double* x,
		std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){

	// make flexible for different numbers of images for different cameras ....
	Problem problem;

	int number_cameras = COs.size();
	// all of them will have the same size, null matrices will be in places where the the camera does not view the pattern.
	int number_images = Bs.size();
	int number_points = COs[0].all_3d_corners[0].size();
	// 2 residuals for each point and each image and each camera.


	vector<vector<double*> > threeDpoints;
	vector<vector<double*> > twoDpoints;
	for (int i = 0; i < number_cameras; i++){
		threeDpoints.push_back(vector<double*>());
		twoDpoints.push_back(vector<double*>());

		for (int j = 0; j < number_images; j++){
			threeDpoints[i].push_back(new double[3*number_points]);
			twoDpoints[i].push_back(new double[2*number_points]);
		}
	}

	double* B = new double[16*number_images];

	vector<double*> parameter_blocks;

	int minimum_number = number_images;

	vector<int> number_of_images_per_camera;
	vector<double> weighting;


	///////////////////////// WEIGHTING FOR MISSING IMAGES /////////////////////////////////////
	// count number for each camera ....
	for (int j = 0; j < number_cameras; j++){
		int count = 0;

		for (int i = 0; i < number_images; i++){
			if (COs[j].all_points[COs[j].number_internal_images_written + i].size() > 0){
				count++;
			}
		}

		number_of_images_per_camera.push_back(count);

		if (count < minimum_number){
			minimum_number = count;
		}
	}

	out << "Camera number, number of images per camera, weighting " << endl;
	for (int j = 0; j < number_cameras; j++){
		weighting.push_back(double(minimum_number)/double(number_of_images_per_camera[j]));
		out << j << " " << number_of_images_per_camera[j] << " " << weighting[j] << endl;
	}
	out << endl;

	////////////////// Put in appropriate format --  /////////////////////////////////////////////////////////////////////////
	// fill in camera_params, internal and then 8 radial distortion paramerters.

	// we want to take back x_reproject
	CopyFromCalibration(COs, camera_params);

	for (int i = 0; i < number_cameras; i++){
		for (int j = 0; j < 7; j++){
			parameter_blocks.push_back(&x[i*7 + j]);
		}
//		// first row
//		camera_params[12*i] = COs[i].A[0][0];
//		camera_params[12*i + 1] = COs[i].A[0][2];
//		camera_params[12*i + 2] = COs[i].A[1][1];
//		camera_params[12*i + 3] = COs[i].A[1][2];
//
//		for (int j = 0; j < 8; j++){
//			camera_params[12*i + 4 + j] = COs[i].k[j];
//		}

		for (int j = 0; j < 12; j++){
			parameter_blocks.push_back(&camera_params[12*i + j]);
		}
	}

	for (int j = 0; j < 7; j++){
		parameter_blocks.push_back(&x[number_cameras*7 + j]);
	}



	//19 vs 26


	out << "Preliminary" << endl;
	out << "X  ... Z" << endl;
	for (int j = 0; j < number_cameras + 1; j++){
		for (int i = 0; i < 7; i++){
			out << x[7*j + i] << " ";
		}
		out << endl;
	};


	out << "camera cali parameters: " << endl;
	for (int i = 0; i < number_cameras; i++){
		for (int j = 0; j < 12; j++){
			out << camera_params[12*i + j] << " ";
		}
		out << endl;
	}
	out << endl;
	out << endl;

	for (int j = 0; j < number_cameras; j++){
		int non_blanks = 0;
		for (int i = 0; i < number_images; i++) {
			//for (int i = 0; i < 1; i++) {

			if (COs[j].all_points[COs[j].number_internal_images_written + i].size() > 0){
				// for each image, B stays the same
				for (int r = 0, in = 0; r < 4; r++){
					for (int c = 0; c < 4; c++, in++){
						B[in + 16*i] = Bs[i](r + 1, c + 1);
					}
				}


				for (int k = 0; k < number_points; k++){
					//for (int k = 0; k < 10; k++){
					twoDpoints[j][i][k*2] = COs[j].all_points[COs[j].number_internal_images_written + i][k].x;
					twoDpoints[j][i][k*2 + 1] = COs[j].all_points[COs[j].number_internal_images_written + i][k].y;

					threeDpoints[j][i][k*3] = COs[j].all_3d_corners[COs[j].number_internal_images_written + non_blanks][k].x;
					threeDpoints[j][i][k*3 + 1] = COs[j].all_3d_corners[COs[j].number_internal_images_written + non_blanks][k].y;
					threeDpoints[j][i][k*3 + 2] = COs[j].all_3d_corners[COs[j].number_internal_images_written + non_blanks][k].z;

					//cout << "Before create " << endl; //char fg; cin >> fg;

					ceres::CostFunction* cost_function =
							RP1_2_multi::Create(
									&camera_params[12*j], &B[16*i], &twoDpoints[j][i][k*2], &threeDpoints[j][i][k*3],
									param_type, cost_type, number_points, number_cameras, j, &weighting[j]);
					//cout << "After create " << endl; //cin >> fg;
					problem.AddResidualBlock(cost_function,
							NULL /* squared loss */,
							parameter_blocks);
					//cout << "After add " << endl; //cin >> fg;
				}

				//cout << "Before create " << endl; //char fg; cin >> fg;

				ceres::CostFunction* cost_function =
						RP1_2_multi::Create(
								&camera_params[12*j], &B[16*i], &twoDpoints[j][i][0], &threeDpoints[j][i][0], param_type, cost_type, number_points, number_cameras, j, &weighting[j]);
				//cout << "After create " << endl; //cin >> fg;
				problem.AddResidualBlock(cost_function,
						NULL /* squared loss */,
						parameter_blocks);
				//cout << "After add " << endl; //cin >> fg;

				non_blanks++;
			}
		}
	}


	//cout << "After create blocks " << endl; char fg; cin >> fg;


	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	out << summary.FullReport() << endl;
	std::cout << summary.BriefReport() << "\n";

	cout << "After running solver " << endl;
	//char ch; cin >> ch;

	out << "X  ... Z" << endl;
	for (int j = 0; j < number_cameras + 1; j++){
		for (int i = 0; i < 7; i++){
			out << x[7*j + i] << " ";
		}
		out << endl;
	};
	out << endl;


	out << "camera cali parameters: " << endl;
	for (int i = 0; i < number_cameras; i++){
		for (int j = 0; j < 12; j++){
			out << camera_params[12*i + j] << " ";
		}
		out << endl;
	}
	out << endl;


	parameter_blocks.clear();
	//delete [] A;
	for (int i = 0; i < number_cameras; i++){
		for (int j = 0; j < number_images; j++){
			delete [] twoDpoints[i][j];
			delete [] threeDpoints[i][j];
		}
	}
	delete [] B;
}

