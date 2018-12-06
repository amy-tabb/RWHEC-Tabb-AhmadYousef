// This code is being distributed with no warranties whatsoever.
//
// Author: Amy Tabb.  Original release circa 2016.  Current date May 30, 2018.
// small updates Dec. 3 2018
//
//  If you use this code to calibrate a robot in an academic setting, please cite this paper!  Thanks a bunch.
//@article{tabb_solving_2017,
//	title = {Solving the robot-world hand-eye(s) calibration problem with iterative methods},
//	volume = {28},
//	issn = {1432-1769},
//	url = {https://doi.org/10.1007/s00138-017-0841-7},
//	doi = {10.1007/s00138-017-0841-7},
//	abstract = {Robot-world, hand-eye calibration is the problem of determining the transformation between the robot end-effector and a camera, as well as the transformation between the robot base and the world coordinate system. This relationship has been modeled as                                                                           \$\$\{{\textbackslash}mathbf \{AX\}\}=\{{\textbackslash}mathbf \{ZB\}\}\$\$                                                                                    AX                        =                        ZB                                                                            , where                                                                           \$\$\{{\textbackslash}mathbf \{X\}\}\$\$                                                            X                                                       and                                                                           \$\$\{{\textbackslash}mathbf \{Z\}\}\$\$                                                            Z                                                       are unknown homogeneous transformation matrices. The successful execution of many robot manipulation tasks depends on determining these matrices accurately, and we are particularly interested in the use of calibration for use in vision tasks. In this work, we describe a collection of methods consisting of two cost function classes, three different parameterizations of rotation components, and separable versus simultaneous formulations. We explore the behavior of this collection of methods on real datasets and simulated datasets and compare to seven other state-of-the-art methods. Our collection of methods returns greater accuracy on many metrics as compared to the state-of-the-art. The collection of methods is extended to the problem of robot-world hand-multiple-eye calibration, and results are shown with two and three cameras mounted on the same robot.},
//	number = {5},
//	journal = {Machine Vision and Applications},
//	author = {Tabb, Amy and Ahmad Yousef, Khalil M.},
//	month = aug,
//	year = {2017},
//	pages = {569--590}
//}
//
// If you've inherited this code, the original source is from the github repository amy-tabb .  Check back there for updates periodically, particularly as it pertains to OpenCV versions.
// You're welcome to modify it to your needs for other projects.

#include "Tabb_AhmadYousef_RWHEC_Jun2018_main.hpp"

#include "DirectoryFunctions.hpp"
#include "StringFunctions.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

using namespace std;

bool VERBOSE = true;


int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	Eigen::initParallel();

	/////////////////////////////************************************/////////////////////////////
	string source_dir;
	string external_dir;
	string write_dir;
	string robot_file = "";
	string robot_dir;
	string hand_eye_dir;
	string cali_object_file;


	double  chess_mm_height = 0;
	double chess_mm_width = 0;
	int chess_height = 0;
	int chess_width = 0;
	int flag;
	bool do_camcali = true;
	bool do_rwhec = true;
	bool do_reconstruction = true;

	cout << "Usage for calibration is function_name input_directory write_directory camcali_rwhec flag (no flag -- do both, flag == 0, do camera only camera calibration, flag == 1, do only rwhec). Full path is needed for robot_directory " << endl;

	if (argc >= 3){
		source_dir =   string(argv[1]);
		write_dir = argv[2];

		EnsureDirHasTrailingBackslash(source_dir);

		cali_object_file =  string(source_dir) + "calibration_object.txt";
		if (argc == 4){
			flag = FromString<int>(argv[3]);
			do_camcali = false;
			do_rwhec = false;
			do_reconstruction = false;

			switch (flag) {
			case 0: {
				do_camcali = true;
			} break;
			case 1: {
				do_rwhec = true;
				do_reconstruction = true;
			} break;
			default: {
				cout << "Bad flag -- not 0-1  " << endl;
				exit(1);
			}
			}
		}
	}	else {
		cout << "Wrong number of arguments " << endl;
		exit(1);
	}

	ifstream in;
	in.open(cali_object_file.c_str());

	if (!in){
		cout << "You forgot the calibration_object.txt file or the path is wrong" << endl << argv[1] << endl;;
		cout << cali_object_file << endl;
		exit(1);
	}
	in.close();


//	string temp;
//	in >> temp >> chess_mm_height;
//	in >> temp >> chess_mm_width;
//	in >> temp >> chess_height;
//	in >> temp >> chess_width;
//	in.close();

	/////////////////////    READ CALIBRATION ITEMS FROM FILE /////////////////
	string fieldString;
	string returnString;

	fieldString = "chess_mm_height";
	returnString = FindValueOfFieldInFile(cali_object_file, fieldString, false);

	if (returnString.size() != 0){
		chess_mm_height = FromString<double>(returnString);
	}	else {
		cout << "Cannot find " << fieldString << "in " << cali_object_file << endl;
		exit(1);
	}


	fieldString = "chess_mm_width";
	returnString = FindValueOfFieldInFile(cali_object_file, fieldString, false);

	if (returnString.size() != 0){
		chess_mm_width = FromString<double>(returnString);
	}	else {
		cout << "Cannot find " << fieldString << "in " << cali_object_file << endl;
		exit(1);
	}


	fieldString = "chess_height";
	returnString = FindValueOfFieldInFile(cali_object_file, fieldString, false);

	if (returnString.size() != 0){
		chess_height = FromString<int>(returnString);
	}	else {
		cout << "Cannot find " << fieldString << "in " << cali_object_file << endl;
		exit(1);
	}


	fieldString = "chess_width";
	returnString = FindValueOfFieldInFile(cali_object_file, fieldString, false);

	if (returnString.size() != 0){
		chess_width = FromString<int>(returnString);
	}	else {
		cout << "Cannot find " << fieldString << "in " << cali_object_file << endl;
		exit(1);
	}


	RobotWorldHandEyeCalibration(chess_mm_width, chess_mm_height, chess_height, chess_width, source_dir, write_dir, do_camcali, do_rwhec, do_reconstruction, VERBOSE);

	return 0;
}


int RobotWorldHandEyeCalibration(double square_mm_height, double square_mm_width,
		int chess_h, int chess_w, string source_dir, string write_dir, bool do_camcali, bool do_rwhec, bool do_reconstruction, bool verbose){

	string command;
	//int ret;
	int robot_mounted_cameras = 0;
	vector<string> camera_names;
	string image_dir;
	vector<string> window_names; // is this used?
	string name;
	std::ofstream out;
	std::ifstream in;
	string filename;

	// first determine the number of cameras involved
	ReadDirectory(source_dir + "images", camera_names);
	robot_mounted_cameras = camera_names.size();


	if (verbose){
		cout << "Number of cameras " << robot_mounted_cameras << endl;
	}

	string internal_dir;
	string external_dir = source_dir + "/images";

	vector<vector<MatrixXd> > As;
	vector<Matrix4d> Bs;
	int number_cameras;

	vector<CaliObjectOpenCV2> COs;
	vector<int> number_images_per(robot_mounted_cameras, 0);
	int total_number_images_all_cameras = 0;

	//char ch;

	if (do_camcali){
		for (int i = 0; i < robot_mounted_cameras; i++){
			command = "mkdir " + write_dir + "camera_results" + ToString<int>(i);
			int ret = system(command.c_str());
		}

		for (int k = 0; k < robot_mounted_cameras; k++){
			cout << "calibrating number " << k << endl;
			/////////////////////// Camera calibration ....///////////////////////////////////////////
			filename = write_dir + "camera_results" + ToString<int>(k) + "/details.txt";
			out.open(filename.c_str());

			COs.push_back(CaliObjectOpenCV2(0, chess_w, chess_h,
					square_mm_width, square_mm_height));

			external_dir = source_dir + "images/" + camera_names[k] ;
			internal_dir = source_dir + "internal_images/" + camera_names[k];

			DIR* dir = opendir(internal_dir.c_str());
			if (dir)
			{
				/* Directory exists. */
				closedir(dir);
				COs[k].ReadImages(internal_dir, 0);
			}

			COs[k].ReadImages(external_dir, 1);

			// argument is whether or not to draw the corners ....
			COs[k].AccumulateCornersFlexibleExternal(true);

			COs[k].CalibrateFlexibleExternal(out,  write_dir + "camera_results" + ToString<int>(k));
			out.close();

			// write cali file
			filename = write_dir + "camera_results" + ToString<int>(k) + "/cali.txt";
			out.open(filename.c_str());

			WriteCaliFile(&COs[k], out);

			out.close();


			MatrixXd A1(4, 4);
			MatrixXd B1(4, 4);
			MatrixXd temp_matrix(0, 0);


			As.push_back(vector<MatrixXd>());

			number_cameras = COs[k].Rts.size();

			// convert the external parameters from the camera calibration process into the A matrices
			for (int i = 0; i < int(COs[k].Rts.size()); i++){
				//cout << "i " << i << endl;
				if (COs[k].Rts[i].size() > 0){
					A1.setIdentity();

					for (int r = 0; r < 3; r++){
						for (int c = 0; c < 4; c++){
							A1(r, c) = COs[k].Rts[i][r][c];
						}
					}
					As[k].push_back(A1);
					number_images_per[k]++;
				}	else {
					As[k].push_back(temp_matrix);
				}
			}

			total_number_images_all_cameras += number_images_per[k];
			out.close();
		}


		// need to write
		for (int k = 0; k < robot_mounted_cameras; k++){
			filename = write_dir + "/camera_results" + ToString<int>(k) + "/program_readable_details.txt";
			out.open(filename.c_str());

			out << int(COs[k].Rts.size()) << endl;

			// write the internal matrix
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++){
					out << std::setprecision(9) << COs[k].A[i][j] << " ";
				}
			}
			out << endl;

			for (int i = 0; i < 8; i++){
				out << COs[k].k[i] << " ";
			}
			out << endl;

			// first write which ones have the pattern present
			for (int i = 0; i < int(COs[k].Rts.size()); i++){
				if (COs[k].Rts[i].size() > 0){
					out << "1 ";
				}	else {
					out << "0 ";
				}
			}
			out << endl;

			// write the 3D points just to make it easier to read later.
			int number_points = COs[k].all_3d_corners[0].size();

			for (int j = 0; j < number_points; j++){
				out << COs[k].all_3d_corners[COs[k].number_internal_images_written][j].x << " " << COs[k].all_3d_corners[COs[k].number_internal_images_written][j].y << " ";
				out << COs[k].all_3d_corners[COs[k].number_internal_images_written][j].z  << " ";
			}
			out << endl;

			// write the As and image points
			for (int i = 0; i < int(COs[k].Rts.size()); i++){
				if (As[k][i].rows() > 0){
					out << As[k][i] << endl;

					for (int j = 0; j < number_points; j++){
						out << COs[k].all_points[COs[k].number_internal_images_written + i][j].x << " " << COs[k].all_points[COs[k].number_internal_images_written + i][j].y << " ";
					}
					out << endl;
				}
			}
			out.close();
		}
	}


	////////////// Transition if both parts -- camera calibration and RWHEC are not selected
	if (!do_rwhec && !do_reconstruction){
		cout << "Exiting because we're not doing rwhec OR reconstruction" << endl;
		exit(1);
	}	else {
		if (!do_camcali){
			// need to load everything
			for (int k = 0; k < robot_mounted_cameras; k++){
				/////////////////////// Camera calibration ....///////////////////////////////////////////
				filename = write_dir + "/camera_results" + ToString<int>(k) + "/program_readable_details.txt";
				in.open(filename.c_str());

				COs.push_back(CaliObjectOpenCV2(0, chess_w, chess_h,
						square_mm_width, square_mm_height));

				external_dir = source_dir + "/images/" + camera_names[k] ;
				internal_dir = source_dir + "/internal_images/" + camera_names[k];

				COs[k].ReadImages(external_dir, 1);

				COs[k].image_size = COs[k].external_images[0].size();


				int number_images;
				int flag_present;
				int corner_count = chess_h*chess_w;

				in >> number_images;

				cout << "Number images " << number_images << endl;

				COs[k].A.resize(3, vector<double>(3, 1));

				for (int i = 0; i < 3; i++){
					for (int j = 0; j < 3; j++){
						in >>  COs[k].A[i][j];
					}
				}

				COs[k].k.resize(8, 0);
				for (int i = 0; i < 8; i++){
					in >> COs[k].k[i];
				}

				vector< cv::Point3f> corners_3d(chess_h*chess_w);
				vector<cv::Point2f> image_points(corner_count);

				Matrix4d m4;  m4.setIdentity();
				As.push_back(vector<MatrixXd>());

				vector<bool> saved_flag;

				for (int i = 0; i < number_images; i++){
					in >> flag_present;

					saved_flag.push_back(flag_present);

					if (flag_present == true){
						As[k].push_back(m4);
						COs[k].all_points.push_back(image_points);
						number_images_per[k]++;
					}	else {
						MatrixXd tempzero(0, 0);
						As[k].push_back(tempzero);
						COs[k].all_points.push_back(vector<cv::Point2f>());
					}
				}


				for (int i = 0; i < corner_count; i++){
					in >> corners_3d[i].x >> corners_3d[i].y >> corners_3d[i].z;
				}

				for (int i = 0; i < number_images; i++){
					if (saved_flag[i] == true){
						COs[k].all_3d_corners.push_back(corners_3d);
					}
				}

				// read As and image points.
				for (int i = 0; i < number_images; i++){
					if (saved_flag[i] == true){

						for (int r = 0; r < 4; r++){
							for (int c = 0; c < 4; c++){
								in >>  As[k][i](r, c);
							}
						}

						// read in the points ....
						for (int j = 0; j < corner_count; j++){
							in >> COs[k].all_points[i][j].x >>  COs[k].all_points[i][j].y;
						}
					}
				}


				in.close();

				total_number_images_all_cameras += number_images_per[k];

			}
		}
	}

	//
	/////////////////////////// END CAMERA CALIBRATION ////////////////////////////

	Matrix3d Rz;
	Matrix3d Ry;
	Matrix3d Rx;

	////////////////////////// ROBOT SECTION /////////////////////////////////////////

	filename = source_dir + "/robot_cali.txt";
	FILE * file;
	file = fopen(filename.c_str(), "r");
	if (!file){
		cout << "Robot file is null -- fix and exiting for now... " << filename << endl;
		exit(1);
	}	else {
		 fclose(file);
		ReadRobotFileRobotCaliTxt(filename, Bs);
	}


	Matrix3d TestR;
	Matrix4d X;
	Matrix4d Z;

	X.setIdentity();  Z.setIdentity();


	double null_triple[3];
	null_triple[0] = 0;
	null_triple[1] = 0;
	null_triple[2] = 0;
	double* xarray = new double[7*(robot_mounted_cameras + 1)];

	// 19 per camera: 4 internals, 8 radial distortion, 7 for X (each) and 7 for Z
	double* xarray_reproj = new double[19*(robot_mounted_cameras) + 7];
	double* camera_parameters = new double[12*robot_mounted_cameras];
	double* camera_parameters_from_cali = new double[12*robot_mounted_cameras];
	double xm[16];
	double* threeDoriginal = new double[3*chess_h*chess_w];

	vector<double> rotation_error;
	vector<double> translation_error;
	vector<double> whole_error;
	vector<vector<double> > reprojection_error;

	vector<double> axis_angle_difference;
	vector<double> reconstruction_reprojection_errors;
	vector<double> difference_between_reconstruction_and_original;
	vector<string> name_vector;

	string descriptor_string;
	string abbreviated_descriptor_string;
	string write_result_directory;
	vector<Matrix4d> Zs;
	PARAM_TYPE param_type;
	COST_TYPE cost_type;


	CopyFromCalibration(COs, camera_parameters_from_cali);
	Initialize3DPoints(COs, threeDoriginal, chess_h*chess_w);

	for (int i = 0; i < robot_mounted_cameras; i++){
		//Xs.push_back(X);
		Zs.push_back(Z);
	}

	bool separable = false;

	vector<Matrix4d> AsComps;
	vector<Matrix4d> BsComps;

	if (robot_mounted_cameras == 1){
		for (int i = 0; i < int(As[0].size()); i++){
			if (As[0][i].rows() > 0){
				AsComps.push_back(As[0][i]);
				BsComps.push_back(Bs[i]);
			}
		}
	}


	string reconstruction_dir;
	if (do_reconstruction){

		reconstruction_dir =  write_dir + "/reconstructions";

		command = "mkdir " + reconstruction_dir;
		system(command.c_str());
	}


	// All of the methods -- this is optional.  You could choose one or two methods to save time, for instance, a reprojection error method is recommended.
	// (suggest any of 12-16)
	for (int option = 0; option < 18; option++ )
	{

		separable = false;
		switch (option) {
		case 0:{
			descriptor_string = "Euler param. c1 simultaneous ";
			abbreviated_descriptor_string = "E_c1_simul";
			param_type = Euler;
			cost_type = c1;
			separable = false;
		} break;
		case 1:{
			descriptor_string = "Axis Angle c1 simultaneous";
			abbreviated_descriptor_string = "AA_c1_simul";
			param_type = AxisAngle;
			cost_type = c1;
		} break;
		case 2:{
			descriptor_string = "Quaternion c1 simultaneous ";
			abbreviated_descriptor_string = "Q_c1_simul";
			param_type = Cali_Quaternion;
			cost_type = c1;
			separable = false;
		} break;
		case 3:{
			descriptor_string = "Euler param. c2 simultaneous";
			abbreviated_descriptor_string = "E_c2_simul";
			param_type = Euler;
			cost_type = c2;
			separable = false;
		} break;
		case 4:{
			descriptor_string = "Axis Angle c2 simultaneous";
			abbreviated_descriptor_string = "AA_c2_simul";
			param_type = AxisAngle;
			cost_type = c2;
			separable = false;
		} break;
		case 5:{
			descriptor_string = "Quaternion c2 simultaneous";
			abbreviated_descriptor_string = "Q_c2_simul";
			param_type = Cali_Quaternion;
			cost_type = c2;
			separable = false;
		} break;
		case 6:{
			descriptor_string = "Euler parameterzation c1 separable ";
			abbreviated_descriptor_string = "E_c1_separable";
			param_type = Euler;
			cost_type = c1;
			separable = true;
		} break;
		case 7:{
			descriptor_string = "Axis Angle c1 separable ";
			abbreviated_descriptor_string = "AA_c1_separable";
			param_type = AxisAngle;
			cost_type = c1;
			separable = true;
		} break;
		case 8:{
			descriptor_string = "Quaternion c1 separable ";
			abbreviated_descriptor_string = "Q_c1_separable";
			param_type = Cali_Quaternion;
			cost_type = c1;
			separable = true;
		} break;
		case 9:{
			descriptor_string = "Euler param. c2 separable ";
			abbreviated_descriptor_string = "E_c2_separable";
			param_type = Euler;
			cost_type = c2;
			separable = true;
		} break;
		case 10:{
			descriptor_string = "Axis Angle c2 separable ";
			abbreviated_descriptor_string = "AA_c2_separable";
			param_type = AxisAngle;
			cost_type = c2;
			separable = true;
		} break;
		case 11:{
			descriptor_string = "Quaternion c2 separable ";
			abbreviated_descriptor_string = "Q_c2_separable";
			param_type = Cali_Quaternion;
			cost_type = c2;
			separable = true;
		} break;
		case 12:{
			descriptor_string = "Euler, Reprojection Error I ";
			abbreviated_descriptor_string = "E_RPI";
			param_type = Euler;
			cost_type = rp1;
		} break;
		case 13:{
			descriptor_string = "Axis Angle, Reprojection Error I";
			abbreviated_descriptor_string = "AA_RPI";
			param_type = AxisAngle;
			cost_type = rp1;
		} break;
		case 14:{
			descriptor_string = "Quaternion, Reprojection Error I";
			abbreviated_descriptor_string = "Q_RPI";
			param_type = Cali_Quaternion;
			cost_type = rp1;
		} break;
		case 15:{
			descriptor_string = "Euler, Reprojection Error II ";
			abbreviated_descriptor_string = "E_RPII";
			param_type = Euler;
			cost_type = rp2;
		} break;
		case 16:{
			descriptor_string = "AxisAngle, Reprojection Error II ";
			abbreviated_descriptor_string = "AA_RPII";
			param_type = AxisAngle;
			cost_type = rp2;
		} break;
		case 17:{
			descriptor_string = "Quaternion, Reprojection Error II ";
			abbreviated_descriptor_string = "Q_RPII";
			param_type = Cali_Quaternion;
			cost_type = rp2;
		} break;
		}

		write_result_directory = write_dir + "/rwhe_" + abbreviated_descriptor_string;
		name_vector.push_back(descriptor_string);

		/// create directories  //////////////////////////////////

		if (do_rwhec){

			command = "mkdir " + write_result_directory;
			system(command.c_str());

			filename = write_result_directory + "/details.txt";

			out.open(filename.c_str());


			switch (cost_type){
			case c1: {
				 // This case intentionally left blank
			}
			case c2: {
				switch (param_type){
				case Euler: {
					out << "Euler parameterization  ";
				} break;
				case AxisAngle: {
					out << "Axis Angle parameterization  ";
				} break;
				case Cali_Quaternion: {
					out << "Quaternion parameterization  ";
				} break;
				}

				switch (cost_type){
				case c1: {
					out <<  " ||AX-ZB||^2, no reprojection error" << endl;
				} break;
				case c2: {
					out << " ||A-ZBX^{-1}||^2, no reprojection error" << endl;
				} break;
				default: {
					cout << "Other types not handled in this switch." << endl;
				} break;
				}


				{

					for (int i = 0; i < (robot_mounted_cameras + 1)*7; i++){
						xarray[i] = 0;
					}

					// x array is x, then zs.
					if (param_type == Cali_Quaternion){
						for (int i = 0; i < (robot_mounted_cameras + 1); i++){
							ceres::AngleAxisToQuaternion(null_triple, &xarray[7*i]);
						}
					}



					// solve
					if (separable == false){
						CF1_2_multi_camera(As, Bs, xarray, out, param_type, cost_type);
					}	else {
						CF1_2_multi_camera_separable(As, Bs, xarray, out, param_type, cost_type, rotation_only);

						CF1_2_multi_camera_separable(As, Bs, xarray, out, param_type, cost_type, translation_only);
					}


					// convert to the matrix representation
					for (int i = 1; i < (robot_mounted_cameras + 1); i++)
					{


						switch (param_type){
						case Euler: {
							Convert6ParameterEulerAngleRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
						} break;
						case AxisAngle: {
							Convert6ParameterAxisAngleRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
						} break;
						case Cali_Quaternion: {
							Convert7ParameterQuaternionRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
						} break;
						}


						for (int r = 0, in = 0; r < 4; r++){
							for (int c = 0; c < 4; c++, in++){
								Zs[i - 1](r, c) = xm[in];
							}
						}

					}

					switch (param_type){
					case Euler: {
						Convert6ParameterEulerAngleRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
					} break;
					case AxisAngle: {
						Convert6ParameterAxisAngleRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
					} break;
					case Cali_Quaternion: {
						Convert7ParameterQuaternionRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
					} break;
					}


					// convert to the X, Z representation
					for (int r = 0, in = 0; r < 4; r++){
						for (int c = 0; c < 4; c++, in++){
							X(r, c) = xm[in];
						}
					}

					if (cost_type == c2){
						X = X.inverse().eval();
					}


				}
			} break;
			case rp1: {
				/// no break at the end intentional
			}
			case rp2: {
				switch (param_type){
				case Euler: {
					out << "Euler parameterization  ";
				} break;
				case AxisAngle: {
					out << "Axis Angle parameterization  ";
				} break;
				case Cali_Quaternion: {
					out << "Quaternion parameterization  ";
				} break;
				}

				switch (cost_type){
				case rp1: {
					out <<  " Reprojection Error, no camera parameters" << endl;
				} break;
				case rp2: {
					out << " Reprojection Error, yes camera parameters" << endl;
				} break;
				default: {
					cout << "Other types not handles in this switch." << endl;
				} break;
				}

				for (int i = 0; i < (robot_mounted_cameras + 1)*7; i++){
					xarray[i] = 0;
				}

				if (param_type == Cali_Quaternion){
					for (int i = 0; i < (robot_mounted_cameras + 1); i++){
						ceres::AngleAxisToQuaternion(null_triple, &xarray[7*i]);
					}
				}


				// solve, initial solution
				CF1_2_multi_camera(As, Bs, xarray, out, param_type, c2);


				// now x is x inverse b/c we used c2.  Ahat = ZBX^-1
				if (cost_type == rp2){
					RP1_2_multi_camera_sparse(COs, Bs, camera_parameters, xarray, out, param_type, rp1);
				}

				RP1_2_multi_camera_sparse(COs, Bs, camera_parameters, xarray, out, param_type, cost_type);

				// convert to the matrix representation
				for (int i = 1; i < (robot_mounted_cameras + 1); i++)
				{


					switch (param_type){
					case Euler: {
						Convert6ParameterEulerAngleRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
					} break;
					case AxisAngle: {
						Convert6ParameterAxisAngleRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
					} break;
					case Cali_Quaternion: {
						Convert7ParameterQuaternionRepresentationIntoMatrix<double>(&xarray[7*i], &xm[0]);
					} break;
					}


					for (int r = 0, in = 0; r < 4; r++){
						for (int c = 0; c < 4; c++, in++){
							Zs[i - 1](r, c) = xm[in];
						}
					}
				}


				switch (param_type){
				case Euler: {
					Convert6ParameterEulerAngleRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
				} break;
				case AxisAngle: {
					Convert6ParameterAxisAngleRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
				} break;
				case Cali_Quaternion: {
					Convert7ParameterQuaternionRepresentationIntoMatrix<double>(&xarray[0], &xm[0]);
				} break;
				}


				for (int r = 0, in = 0; r < 4; r++){
					for (int c = 0; c < 4; c++, in++){
						X(r, c) = xm[in];
					}
				}

				X = X.inverse().eval();

				if (cost_type == rp2){
					CopyToCalibration(COs, camera_parameters);
				}
			} break;

			}


			cout << "X " << endl << X << endl;
			out << "X " << endl << X << endl;

			for (int i = 0; i < robot_mounted_cameras; i++){
				cout << "Z " << i << endl << Zs[i] << endl;
				out << "Z " << i << endl << Zs[i] << endl;
			}





			double error;
			error  = 0;

			for (int i = 0; i < robot_mounted_cameras; i++){
				cout << "Size " << As[i].size() << ", " << Bs.size() << ", " << Zs.size() << endl;

				error += AssessRotationError(As[i], Bs, X, Zs[i]);
			}
			out << "Summed rotation error " << error << endl;
			rotation_error.push_back(error);

			error  = 0;
			for (int i = 0; i < robot_mounted_cameras; i++){
				cout << "Size " << As[i].size() << ", " << Bs.size() << ", " << Zs.size() << endl;
				error += AssessRotationErrorAxisAngle(As[i], Bs, X, Zs[i]);
			}
			out << "Summed angle difference rotation error " << error << endl;
			axis_angle_difference.push_back(error);

			error  = 0;
			for (int i = 0; i < robot_mounted_cameras; i++){
				error += AssessTranslationError(As[i], Bs, X, Zs[i]);
			}
			out << "Summed translation error " << error << endl;
			translation_error.push_back(error);

			error  = 0;
			for (int i = 0; i < robot_mounted_cameras; i++){
				error += AssessErrorWhole(As[i], Bs, X, Zs[i]);
			}
			out << "Summed whole error " << error << endl;
			whole_error.push_back(error);

			error  = 0;
			vector<double> ID_project_error;
			for (int i = 0; i < robot_mounted_cameras; i++){
				error = CalculateReprojectionError(&COs[i], As[i], Bs, X, Zs[i], out, write_result_directory, i);
				ID_project_error.push_back(error);
			}
			reprojection_error.push_back(ID_project_error);

			out.close();

			filename = write_result_directory + "/transformations.txt";
			out.open(filename.c_str());

			out << "X" << endl << X << endl;
			for (int i = 0; i < robot_mounted_cameras; i++){
				out << "Z " << i << endl << Zs[i] << endl;
			}
			out.close();

			for (int i = 0; i < robot_mounted_cameras; i++){
				command = "mkdir " + write_result_directory + "/camera" + ToString<int>(i);
				system(command.c_str());

				filename = write_result_directory + "/camera" + ToString<int>(i) + "/cali.txt";
				out.open(filename.c_str());

				WriteCaliFile(&COs[i], As[i], Bs, X, Zs[i], out);

				out.close();

			}

			if (cost_type == rp2){
				CopyToCalibration(COs, camera_parameters_from_cali);
			}
		}

		if (do_reconstruction){
			cout << "Doing reconstruction!!!! " << option << endl;

			// reading camera info ....
			string camera_cali_file;

			double* parameters = new double[12*robot_mounted_cameras];
			string temp_string;
			for (int i = 0; i < robot_mounted_cameras; i++){
				camera_cali_file = write_result_directory + "/camera" + ToString<int>(i) + "/cali.txt";

				in.open(camera_cali_file.c_str());

				in >> temp_string >> temp_string >> parameters[12*i] >> temp_string >> parameters[12*i + 1] >> temp_string >> parameters[12*i + 2] >> parameters[12*i + 3];

				for (int j = 0; j < 15; j++){
					in >> temp_string;
				}

				for (int j = 0; j < 8; j++){
					in >> parameters[12*i + 4 + j];
				}
				in.close();
			}
			CopyToCalibration(COs, parameters);
			delete [] parameters;

			/// reading transformations.
			string trans_file = write_result_directory + "/transformations.txt";
			in.open(trans_file.c_str());

			in >> temp_string; /// X
			for (int r = 0; r < 4; r++){
				for (int c = 0; c < 4; c++){
					in >> X(r, c);
				}
			}

			for (int i = 0; i < robot_mounted_cameras; i++){
				in >> temp_string >> temp_string;


				for (int r = 0; r < 4; r++){
					for (int c = 0; c < 4; c++){
						in >> Zs[i](r, c);
					}
				}
			}
			in.close();

			// solve for best X given camera parameters and locations of points
			double* threeDpoints = new double[3*chess_h*chess_w];
			double error_from_original  = 0;

			filename = reconstruction_dir + "/" + abbreviated_descriptor_string + ".txt";
			out.open(filename.c_str());


			if (option >= 0){
				ReconstructXFunctionIndividuals(COs, Bs, X, Zs, threeDpoints, reconstruction_reprojection_errors, out);
				error_from_original = ComputeSummedSquaredDistanceBetweenSets(threeDpoints, threeDoriginal, chess_h*chess_w);

				filename = reconstruction_dir + "/" + abbreviated_descriptor_string + ".ply";
				WritePatterns(threeDpoints, chess_h, chess_w, option + 1, filename);
			}	else {
				reconstruction_reprojection_errors.push_back(0);
			}

			difference_between_reconstruction_and_original.push_back(error_from_original/double(chess_h*chess_w));

			out << "Reprojection error " << reconstruction_reprojection_errors.back() << endl;
			out << "RMSE from pattern  " << difference_between_reconstruction_and_original.back() << endl;


			out.close();
			// visualize

			if (option == 0){
				filename = reconstruction_dir + "/ideal.ply";
				WritePatterns(threeDoriginal, chess_h, chess_w, 0, filename);
			}


			// copy back the parameters in case they got muddled.
			CopyToCalibration(COs, camera_parameters_from_cali);

		}
	}

	delete [] xarray;
	delete [] xarray_reproj;
	delete [] camera_parameters;
	delete [] camera_parameters_from_cali;
	delete [] threeDoriginal;

	if (do_rwhec){
		filename = write_dir + "/comparisons.txt";
		out.open(filename.c_str());
		out << "Errors are normalized by the number of robot positions, which is " << number_cameras << " summed for all cameras, which is " << robot_mounted_cameras << " excepting the rms" << endl;
		out << setw(36) << "Method name "
				<< setw(20) << "Rotation error"
				<< setw(20) << "Rot error - angle"
				<< setw(20) << "Translation error"
				<< setw(20) << "Whole error"
				<< setw(20*robot_mounted_cameras) << "reprojection error"
				<< setw(20*robot_mounted_cameras) << "RMS reprojection" << endl;

		std::cout.precision(6);

		for (int i = 0; i < int(rotation_error.size()); i++){
			out << i << " " << setw(36);
			out << name_vector[i];
			out <<  " " << rotation_error[i]
				 << " " << setw(20) << axis_angle_difference[i]
																				   << setw(20) <<  translation_error[i];

			out << setw(20) << whole_error[i];

			for (int j = 0; j < robot_mounted_cameras; j++){
				out <<  setw(20) << reprojection_error[i][j];
			}


			for (int j = 0; j < robot_mounted_cameras; j++){
				out <<  setw(20) << sqrt((1.0/double(As.size()*number_images_per[j]*COs[0].all_3d_corners[0].size())) * reprojection_error[i][j]);
			}
			out << endl;
		}


		out << endl;
		out << "___________________________________________________________________________________" << endl;
		out << endl;

		out.close();
	}

	if (do_reconstruction){
		filename = write_dir + "/reconstruction_accuracy_error_comparisons.txt";
		out.open(filename.c_str());

		out << setw(36) << "Method name "
				<< setw(20) << "reprojection error"
				<< setw(20) << "difference distance" <<  endl;

		for (int i = 0; i < int(reconstruction_reprojection_errors.size()); i++){
			out << i << " " << setw(36);
			out << name_vector[i];
			out << setw(40) << reconstruction_reprojection_errors[i]
																  << setw(40) << difference_between_reconstruction_and_original[i]<< endl;
		}
	}


	return 1;
}



void WriteCaliFile(CaliObjectOpenCV2* CO, std::ofstream& out){

	int n = 0;
	for (int i = 0; i < int(CO->Rts.size()); i++){
		if (CO->Rts[i].size() > 0){
			n++;
		}
	}
	out << n << endl;

	for (int i = 0; i < int(CO->Rts.size()); i++){

		if (CO->Rts[i].size() > 0){
			out << "image" << ToString<int>(i) << ".png " << " ";
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					out << CO->A[r][c] << " ";
				}
			}

			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					out << CO->Rts[i][r][c] << " ";;
				}
			}

			for (int r = 0; r < 3; r++){
				out << CO->Rts[i][r][3] << " ";;
			}

			for (int r = 0; r < int(CO->k.size()); r++){
				out << CO->k[r] << " ";
			}

			out << endl;
		}
	}
}

void WriteCaliFile(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out){
	out << As.size() << endl;

	Matrix4d newA(4, 4);


	vector<Matrix4d> newAs;
	int n = As.size();
	for (int i = 0; i < n; i++){

		newA = Z*Bs[i]*X.inverse();

		out << "image" << ToString<int>(i) << ".png " << " ";
		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				out << CO->A[r][c] << " ";
			}
		}

		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				out << newA(r,c) << " ";
			}
		}

		for (int r = 0; r < 3; r++){
			out << newA(r,3) << " ";
		}

		for (int r = 0; r < int(CO->k.size()); r++){
			out << CO->k[r] << " ";
		}

		out << endl;

	}
}


void ReadRobotFileRobotCaliTxt(string filename, vector<Matrix4d>& Bs){

//	Matrix3d newR;

//	ColumnVector qt(4);
//	ColumnVector cv(3);
//	ColumnVector rv(3);


//	Matrix Rx(3, 3);
//	Matrix Ry(3, 3);
//	Matrix Rz(3, 3);

	Matrix4d B1;

	std::ifstream in;

	in.open(filename.c_str());
//
//	ColumnVector r_v(6);
//	vector<ColumnVector> robot_params;
//	ColumnVector C(3);


	int N;
	in >> N;

	for (int i = 0; i < N; i++){

		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){
				in >> B1(r, c);
			}
		}

		Bs.push_back(B1);
	}
	in.close();
}

double AssessErrorWhole(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;
	Matrix4d H;

	cout << "error " << error << endl;
	double number_stops = Bs.size();

	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			H = As[i]*X - Z*Bs[i];

			cout << "H" << H << endl;
			for (int r = 0; r < 4; r++){
				for (int c = 0; c < 4; c++){
					error +=  H(r, c)*H(r, c);
				}
			}
			//cout << "Error after one image " << error << endl;
		}
	}

	return error/number_stops;
}

double AssessRotationError(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;
	double local_error = 0;


	double number_stops = Bs.size();
	Matrix3d RA;  Matrix3d RZ;  Matrix3d RX; Matrix3d RB;
	Matrix3d Rdiff;

		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				RZ(r, c) = Z(r, c);
				RX(r, c) = X(r, c);
			}
		}



	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					RA(r, c) = As[i](r, c);
					RB(r, c) = Bs[i](r, c);
				}
			}

			Rdiff = RA*RX - RZ*RB;
			local_error = 0;
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					local_error += Rdiff(r, c)*Rdiff(r, c);
				}
			}

			error += local_error;
		}
	}

	return error/number_stops;
}

double AssessRotationErrorAxisAngle(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;
	double local_error = 0;

	Matrix3d R0;
	Matrix3d R1;
	Matrix3d R_relative;

	Matrix3d RA;  Matrix3d RZ;  Matrix3d RX; Matrix3d RB;

	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			RZ(r, c) = Z(r, c);
			RX(r, c) = X(r, c);
		}
	}


	double RV[9];
	double aa[3];
	double number_stops = Bs.size();
	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			for (int r = 0; r < 3; r++){
					for (int c = 0; c < 3; c++){
						RA(r, c) = As[i](r, c);
						RB(r, c) = Bs[i](r, c);
					}
			}


			R0 = RA*RX;
			R1 = RZ*RB;

			R_relative = R1.transpose()*R0;
			for (int r = 0, index = 0; r < 3; r++){
				for (int c = 0; c < 3; c++, index++){
					RV[index] = R_relative(r, c);
				}
			}

			ceres::RotationMatrixToAngleAxis(RV, aa);

			local_error = 57.2958 * sqrt(pow(aa[0], 2) + pow(aa[1], 2) + pow(aa[2], 2));

			//R.NormFrobenius()
			error += local_error;
		}
	}

	// remember to average amoung cameras when there is more than one camera.
	return error/number_stops;
}



double AssessTranslationError(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z){
	double error = 0;

	Matrix3d RA;  Matrix3d RZ;
	MatrixXd tx(3, 1);
	MatrixXd ta(3, 1);
	MatrixXd tb(3, 1);
	MatrixXd tz(3, 1);

	Matrix4d H;
//	double number_stops = Bs.size();

//	for (int i = 0; i < int(Bs.size()); i++){
//		if (As[i].size() > 0){
//			H = As[i]*X - Z*Bs[i];
//
//			//cout << "H" << H << endl;
//			for (int r = 0; r < 4; r++){
//				for (int c = 3; c < 4; c++){
//					error +=  H(r, c)*H(r, c);
//				}
//			}
//			//cout << "Error after one image " << error << endl;
//		}
//
//	}


	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			RZ(r, c) = Z(r, c);
		}

		tx(r, 0) = X(r, 3);
		tz(r, 0) = Z(r, 3);
	}

	MatrixXd result0, result1;

	double number_stops = Bs.size();

	Matrix4d H1, H2;

	for (int i = 0; i < int(Bs.size()); i++){
		// try once for now .....
		if (As[i].size() > 0){
			H1 = As[i]*X;
			H2 = Z*Bs[i];


			// this is wrong.  top_row = As[i].SubMatrix(1, 3, 1, 3)*X.SubMatrix(1, 3, 4, 4) + As[i].SubMatrix(1, 3, 4, 4)
			for (int r = 0; r < 3; r++){
					for (int c = 0; c < 3; c++){
						RA(r, c) = As[i](r, c);
					}
					ta(r, 0) = As[i](r, 3);
					tb(r, 0) = Bs[i](r, 3);
			}

			result0 = RA*tx + ta - (RZ*tb + tz);

//			cout << "H1 " << H1 << endl;
//			cout << "H2 " << H2 << endl;
//
//			cout << "result0 " << result0 << endl;
//			cout << "result1 " << result1 << endl;
//
//			char fg; cin >> fg;

			//cout << "translation result " << result0 << endl;

			for (int r = 0; r < 3; r++){
				//error += (H1(r,3) - H2(r, 3))*(H1(r,3) - H2(r, 3));
				error += result0(r, 0)*result0(r, 0);
			}


		}
	}

	// this error is squared within the parentheses
	// Within the main loop, average amoung cameras for multi-camera datasets.
	return (error/number_stops);
}

double CalculateReprojectionError(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out, string directory, int cam_number){


	double reproj_error = 0;
	vector<cv::Point2f> imagePoints2;
	double err;
	Matrix4d newA;

	vector<Matrix4d> newAs;
	cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	cv::Mat im;
	cv::Mat gray;

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cameraMatrix.at<double>(i, j)  = CO->A[i][j];
		}
	}

	for (int i = 0; i < 8; i++){
		distCoeffs.at<double>(i, 0) = CO->k[i];
	}

	string filename;
	int non_blanks = 0;

	for (int i = 0; i < int(As.size()); i++){
		if (CO->all_points[CO->number_internal_images_written + i].size() > 0){

			newA = Z*Bs[i]*X.inverse();

			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					R.at<double>(r, c) = newA(r, c);
				}
				tvec.at<double>(r, 0) = newA(r, 3);
			}

			cv::Rodrigues(R, rvec);

			cv::projectPoints( cv::Mat(CO->all_3d_corners[CO->number_internal_images_written + non_blanks]), rvec, tvec, cameraMatrix,  // project
					distCoeffs, imagePoints2);
			err = cv::norm(cv::Mat(CO->all_points[CO->number_internal_images_written + i]), cv::Mat(imagePoints2), CV_L2);              // difference

			reproj_error        += err*err;                                             // su

			im = CO->external_images[i].clone();
			cv::cvtColor(im, gray, CV_BGR2GRAY);
			cv::cvtColor(gray, im, CV_GRAY2BGR);


			for (int j = 0; j < int(imagePoints2.size()); j++){
				cv::line(im, imagePoints2[j], CO->all_points[CO->number_internal_images_written + i][j], cv::Scalar(255, 0, 0), 2, 8);
			}


			filename = directory + "/reproj" + ToString<int>(cam_number) + "_" + ToString<int>(i) + ".png";


			cv::imwrite(filename.c_str(), im);
			non_blanks++;
		}

	}

	out << endl << "Summed reproj error "  << reproj_error << endl << endl;


	for (int i = 0; i < int(As.size()); i++){

		newA = Z*Bs[i]*X.inverse();


		out << "newA for i " << i <<  endl << newA << endl;
	}

	return reproj_error;
}


void WritePatterns(double* pattern_points, int chess_h, int chess_w, int index_number, string outfile){

	// each vertex needs a color ....
	// first, find the range ....

	vector<vector<int> > colors;

	vector<int> c(3);
	// lowest value is grey
	c[0] = 0;
	c[1] = 0;
	c[2] = 0;

	colors.push_back(c);

	// next lowest value is purple
	c[0] = 128;
	c[1] = 0;
	c[2] = 128;
	colors.push_back(c);

	// next lowest value is blue
	c[0] = 0;
	c[1] = 0;
	c[2] = 200;
	colors.push_back(c);

	// next lowest value is cyan
	c[0] = 0;
	c[1] = 255;
	c[2] = 255;
	colors.push_back(c);

	// next lowest value is green
	c[0] = 0;
	c[1] = 255;
	c[2] = 0;
	colors.push_back(c);

	// next lowest value is yellow
	c[0] = 255;
	c[1] = 255;
	c[2] = 0;
	colors.push_back(c);

	// next lowest value is red
	c[0] = 255;
	c[1] = 0;
	c[2] = 0;
	colors.push_back(c);

	c = colors[index_number % colors.size()];


	cout << "Writing to " << outfile << endl;
	std::ofstream out;
	out.open(outfile.c_str());

	// first row has chess_h - 2 faces ?
	int number_faces = (chess_h/2)*(chess_w/2) + (chess_h/2 - 1)*(chess_w/2 - 1);


	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "element vertex " << chess_h*chess_w << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	out << "property uchar red" << endl;
	out << "property uchar green" << endl;
	out << "property uchar blue" << endl;
	out << "property uchar alpha" << endl;
	out << "element face " << number_faces << endl;
	out << "property list uchar int vertex_indices"<< endl;
	out << "end_header" << endl;

	for (int i = 0; i < chess_h*chess_w; i++){
		out << pattern_points[i*3] << " " << pattern_points[i*3 + 1] << " " << pattern_points[i*3 + 2] << " " << c[0] << " " << c[1] << " " << c[2] << " 175" << endl;
	}


	int p0, p1, p2, p3;
	for (int i = 0; i < chess_h; i++){

		for (int j = 0; j < chess_w/2; j++){
			if (i % 2  == 0){
				p0 = i*chess_w + 2*j;
				p1 = i*chess_w + 2*j + 1;
				p2 = (i + 1)*chess_w + 2*j + 1;
				p3 = (i + 1)*chess_w + 2*j;
				out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;

			}	else {
				if (j < chess_w/2 - 1){
					p0 = i*chess_w + 2*j + 1;
					p1 = i*chess_w + 2*j + 2;
					p2 = (i + 1)*chess_w + 2*j + 2;
					p3 = (i + 1)*chess_w + 2*j + 1;
					out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;

				}
			}


		}
	}


	out << endl;

	out.close();
}

string FindValueOfFieldInFile(string filename, string fieldTag, bool seperator){

	/// reopen file each time, in case things get switched around.  Assume that these are very small files, not the most efficient.

	ifstream in(filename.c_str());

	if (!in.good()){
		cout << "Filename to find " << fieldTag << " is bad " << filename << " quitting !" << endl;
		exit(1);
	}

	string cmp_str;
	string read_str;


	vector<string> tokens;
	string token;
	string return_str = "";
	bool found = false;


	while (in  && found == false){

		in >> token;

		if (token.compare(fieldTag) == 0){
			found = true;

			if (seperator == true && in){
				in >> token;
			}

			if (in){
				in >> return_str;
			}

		}
	}


	cout << "Found! " << found << " field " << fieldTag << " and result " << return_str << endl;
	in.close();

	return return_str;

}



void EnsureDirHasTrailingBackslash(string& write_directory){
	int n_letters = write_directory.size();
	bool eval =  (write_directory[n_letters - 1] == '/');
	cout << "Last character compare " << write_directory << " " <<  eval << endl;
	if (eval == false){
		write_directory = write_directory + "/";
	}

}

