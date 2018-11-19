/*
 * CostFunctions1_2.cpp
 *
 *  Created on: Nov 18, 2015
 *      Author: atabb
 *      Updated on May25 2018 to use Eigen instead of newmat
 */
#include "CostFunctions1_2.hpp"


void CF1_2_one_camera(vector< vector<Matrix4d> >& As, vector<Matrix4d>& Bs, double* x, double* z, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){

	Problem problem;

	int number_images = As[0].size();
	double* A = new double[16*number_images];
	double* B = new double[16*number_images];


	for (int i = 0; i < int(As[0].size()); i++) {

		for (int r = 0, in = 0; r < 4; r++){
			for (int c = 0; c < 4; c++, in++){
				A[in + 16*i] = As[0][i](r , c);
				B[in + 16*i] = Bs[i](r , c );
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


void CF1_2_multi_camera(vector< vector<Matrix4d> >& As, vector<Matrix4d>& Bs, double* x, std::ofstream& out, PARAM_TYPE param_type, COST_TYPE cost_type){

	// make flexible for different numbers of images for different cameras ....
	Problem problem;

	int number_cameras = As.size();
	// all of them will have the same size, null matrices will be in places where the the camera does not view the pattern.
	int number_images = As[0].size();

	vector<double*> AA;

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
			if (As[j][i].rows() > 0){
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

	out << endl;

	for (int j = 0; j < number_cameras; j++){

		// change all of this newmat stuff out ...
		for (int i = 0; i < number_images; i++) {


			if (As[j][i].size() > 0){
				// for each image, B stays the same
				for (int r = 0, in = 0; r < 4; r++){
					for (int c = 0; c < 4; c++, in++){
						B[in + 16*i] = Bs[i](r , c );
					}
				}


				for (int r = 0, in = 0; r < 4; r++){
					for (int c = 0; c < 4; c++, in++){
						AA[j][16*i + in] = As[j][i](r , c );
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


	parameter_blocks.clear();
	//delete [] A;
	for (int i = 0; i < number_cameras; i++){
		delete [] AA[i];
	}
	delete [] B;
}


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
void RP1_2_multi_camera(vector<CaliObjectOpenCV2>& COs, vector<Matrix4d>& Bs, double* camera_params, double* x,
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
						B[in + 16*i] = Bs[i](r, c);
					}
				}


				for (int k = 0; k < number_points; k++){
					//for (int k = 0; k < 10; k++){
					twoDpoints[j][i][k*2] = COs[j].all_points[COs[j].number_internal_images_written + i][k].x;
					twoDpoints[j][i][k*2 + 1] = COs[j].all_points[COs[j].number_internal_images_written + i][k].y;

					threeDpoints[j][i][k*3] = COs[j].all_3d_corners[COs[j].number_internal_images_written + non_blanks][k].x;
					threeDpoints[j][i][k*3 + 1] = COs[j].all_3d_corners[COs[j].number_internal_images_written + non_blanks][k].y;
					threeDpoints[j][i][k*3 + 2] = COs[j].all_3d_corners[COs[j].number_internal_images_written + non_blanks][k].z;

					ceres::CostFunction* cost_function =
							RP1_2_multi::Create(
									&camera_params[12*j], &B[16*i], &twoDpoints[j][i][k*2], &threeDpoints[j][i][k*3],
									param_type, cost_type, number_points, number_cameras, j, &weighting[j]);
					//cout << "After create " << endl; //cin >> fg;
					problem.AddResidualBlock(cost_function,
							NULL /* squared loss */,
							parameter_blocks);

				}



				ceres::CostFunction* cost_function =
						RP1_2_multi::Create(
								&camera_params[12*j], &B[16*i], &twoDpoints[j][i][0], &threeDpoints[j][i][0], param_type, cost_type, number_points, number_cameras, j, &weighting[j]);

				problem.AddResidualBlock(cost_function,
						NULL /* squared loss */,
						parameter_blocks);


				non_blanks++;
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

	cout << "After running solver " << endl;


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

	for (int i = 0; i < number_cameras; i++){
		for (int j = 0; j < number_images; j++){
			delete [] twoDpoints[i][j];
			delete [] threeDpoints[i][j];
		}
	}
	delete [] B;
}

