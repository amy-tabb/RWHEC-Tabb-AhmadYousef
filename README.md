# RWHEC-Tabb-AhmadYousef

[![DOI](https://zenodo.org/badge/79933846.svg)](https://zenodo.org/badge/latestdoi/79933846)

Comments/Bugs/Problems: amy.tabb@ars.usda.gov

Methods for robot-world, hand-eye calibration; version 2.0.

~June, 2018: This code is up an updated version using the following versions of external libraries:

OpenCV 4.0 (OpenCV 3.0 may work with changes of some enums - read below)

Ceres 1.14

Eigen 3.3.4

A deprecated version using earlier versions of OpenCV (and newmat for matrix handling instead of Eigen) is available https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef2017. 

November 19, 2018: Eclipse project version

December 6, 2018: fix on Eigen version bug.

March 23, 2019: add include wrt OpenCV 4 library to CMakeLists.txt file.

A bug with regards to the computation of the translation error has been corrected. In additon, the code has been cleaned up in this version versus the previous one.  If you wish to understand the code, this is the version to use.  The performance of both versions is identical so far as I can tell on my machines.

# Underlying ideas; how and when to cite this work

This README file is to accompany code for robot-world, hand-eye calibration, produced by Amy Tabb as companion to a paper:
	Solving the Robot-World Hand-Eye(s) Calibration Problem with Iterative Methods

````latex
@Article{tabb_solving_2017,
author="Tabb, Amy
and Ahmad Yousef, Khalil M.",
title="Solving the robot-world hand-eye(s) calibration problem with iterative methods",
journal="Machine Vision and Applications",
year="2017",
month="Aug",
day="01",
volume="28",
number="5",
pages="569--590",
issn="1432-1769",
doi="10.1007/s00138-017-0841-7",
url="https://doi.org/10.1007/s00138-017-0841-7"
}
````

Dataset and/or code:
````latex
@misc{tabb_data_2017,
	title = {Data from: {Solving} the {Robot}-{World} {Hand}-{Eye}(s) {Calibration} {Problem} with {Iterative} {Methods}},
	shorttitle = {Data from},
	url = {https://data.nal.usda.gov/dataset/data-solving-robot-world-hand-eyes-calibration-problem-iterative-methods_3501},
	language = {en},
	urldate = {2019-03-12},
	publisher = {Ag Data Commons},
	author = {Tabb, Amy},
	year = {2017},
	doi = {10.15482/USDA.ADC/1340592},
	note = {type: dataset}
}
````


If you use this code in project that results in a publication, please cite at a minimum the paper above.  Otherwise, there are no restrictions in your use of this code.  However, no guarantees are expressed or implied.

# Building

This README covers instructions for building the code.

## Dependencies

This code uses the Ceres, OpenCV 4.0, and Eigen libraries.  Ceres uses cmake, so as a result, this is a cmake project.  You will need to have cmake installed in order to build the project.

[Ceres](http://ceres-solver.org/)

We are not responsible for whatever it takes to get Ceres to build; but advise that having a recent version of Eigen and Google's glog are helpful to the process. 

[OpenCV](http://opencv.org/) As of this writing, the version installed from [github](https://github.com/opencv/opencv) is returning OpenCV 4.0.

To convert code written under OpenCV 3.0 to OpenCV 4.0, the biggest change revolved around calibration flag enums such as `CV_CALIB_CB_ADAPTIVE_THRESH`.  To get this code to compile under OpenCV 4.0, all I did was change such a flag to `cv::CALIB_CB_ADAPTIVE_THRESH`.  To go back to OpenCV 3.0, you would do the opposite.  I have left the OpenCV 3.x code intact, but commented -- so if you have OpenCV < 4.0, try to compile, find the errors, comment out the OpenCV 4.0 version, and uncomment the OpenCV 3.x versions.  

This code has been tested on Ubuntu 14.04 (for OpenCV 3.x) and Ubuntu 16.04 and Ubuntu 18.04 (for OpenCV 3.x and 4.0).  You are welcome to convert it to Windows, but I have not.  While OpenCV is available from distribution repositories, my long experience with it is has always been to build from the source to get the best results.

**Is getting all of this to work on your system too much of a pain and you are interested in a Docker release?  Let me know!  The squeaky wheel gets the grease.  Email above.**

**Windows:** a user reports that the code compiles on Windows in VisualStudio 2013.  Add the include file `dirent.h`.  29 November 2018.

## Building 

Building:
These instructions assume that you use cmake to generate a project file for the Eclipse Integrated Development Environment (IDE).  There are other methods for using cmake to generate an executable.  You are welcome to use them.  These instructions are provided to give the basics assuming you have an Eclipse installation, and cloned this git repository.  Experienced users, alter at will.

*These instructions have recently been revised, Nov. 19, 2018.*

 1. Clone the git repository.  `cd` to the desired directory, then from the command line  -- ` git clone https://github.com/amy-tabb/RWHEC-Tabb-AhmadYousef.git`.  

2. `$ cd RWHEC-Tabb-AhmadYousef`

3. The code is provided in a directory called `code_src`, along with a `CMakeLists.txt` file.  

4. Then cd to the build directory: from bash `$ cd build`.

5. Then, copy the following string to the base command line: 
```
cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE ../code_src/
```
(if you are doing your own thing, you can replace `../code_src/` with the path to where you put the `code_src` folder.)

6. Then, go to the Eclipse IDE, CDT (C development toolkit). Go to File->Import->Existing projects into workspace. Select the project in the `build` directory.  This will load the project into Eclipse.  Build from Eclipse, and you're ready to run. Note that you may need to check the CMakeLists.txt file for the location of the Eigen and OpenCV includes, and edit if necessary.


## Running

1. The executable using the method above is going to be in the `build` folder.  The arguments for the program are:

	A. input directory -- this is where the images and robot positions are held.

	B. write directory -- this is where all of the results will be written to file, including transformation.

	C. Optional flag, 0 or 1.  If the flag is 0, do only camera calibration.  If the flag is 1, do robot world-hand eye calibration using a previously-computed camera calibration. 
 If no flag is specified, do camera calibration and robot-world, hand-eye calibration.  Most users will want to not specify a flag.

Instructions for the input format can be found in: `README_input_format.txt`.

Run time:
It will take some time (on the order of 4 minutes) to perform camera calibration, and then to perform all of the robot-world, hand-eye calibration methods (4 minutes more), though this varies by dataset and image size. 

# Input format

INPUT FORMAT -- DATASETS ARE POSTED AT THE National Agriculture Library's Ag Data Commons, DOI is [10.15482/USDA.ADC/1340592]( http://dx.doi.org/10.15482/USDA.ADC/1340592)

## Required Directories:

1. `images`

2. `internal_images`

Each of these directories, should contain directories by camera with images.  For instance, for one camera, the `images` directory should contain `camera0`.  For two cameras, it should contain `camera0` and `camera1`. Use the provided datasets as templates for how to set up your own datasets.

The `images` directory represents the images for each stop of the robot.  `internal_images` contains extra images that are used for camera calibration only.  The directory structure of `camera0`, `camera1`, etc. within `internal_images` should be used, if `internal_images` is present.

## Required files:
1. `calibration_object.txt` : This file gives the specification of the calibration pattern.  The units can be changed, however at this time the particular strings used are 'chess_mm_height', 'chess_mm_width' by the program. Make sure that if you are not using millimeters, that you are aware of which units you are using!

	Example from dataset 1:

```
	chess_mm_height 28.5

	chess_mm_width 28.5

	chess_height 6

	chess_width 8
```

2. `robot_cali.txt` : This file gives the number of robot positions, as well as the rotation and translation parameters for the robot.  In the notation of our paper, these matrices are B_i.  Note that our definition is the inverse of some other works.  

3. Optional Included file:
`ExperimentDetails.txt` : This file gives some parameters of the experiments for the provided datasets and is not required to run the code.


# Output format

This README covers the output format generated in the write directory, and how to interpret that format.

## Camera calibration:

For each camera, a directory will be generated with the name `camera_results*` in the write directory, including:

1. For each image that the calibration pattern could be found, an undistorted version of that image.

2. `cali.txt`, a file with the calibration information for each image	

3. `details.txt`, a human-readable file with the internal and external camera calibration information.

4. `program_readable_details.txt`, a file that saves the results of camera calibration to be loaded later by the program for robot-world, hand-eye calibration, if it is desired to perform these in two stages.

## Robot-world, hand-multiple-eye calibration:

A. For each method, a directory will be generated with the short name of the method in the write directory. To correspond directories to the methods in the paper, look at the code in the switch statement (copied at the end of this README).  "rwhe_E_c1_simul" is an abbreviated form of "robot-world, hand-eye calibration using Euler parameterization of rotation components, c1 cost function, simultaneous version."  Each directory contains the following:

1.  A directory for each camera, with the location of the camera as computed by the robot-world, hand-eye calibration method detailed in `cali.txt`.

2. `details.txt`, a log of the progress of the method, including the output from Ceres.

3. `reproj*_*.png`, for each image where the calibration pattern was found, the difference between the reprojected world points of the calibration, as computed using the robot-world, hand-eye calibration X and Z and the original image points is represented with a blue line. In the terminology of our paper, the two points are x_ij-tilde-arrow in Equation 13, and x_ij-arrow.  The sum of the squared line distances for all images in shown in Eq. 14.

4. `transformations.txt`, gives the X and Z matrices for the robot-world, hand-eye calibration method.  If there was more than one camera, there will be more than one Z matrix. 
	
**Note: if you are using one of the methods that also refines the camera calibration parameters, you will have to read that from the `details file`.** 

For instance, these are camera calibration parameters (in vector format), followed by the transformation matrices, for the three-camera case:

```
	camera cali parameters: 
	901.785 614.251 901.953 488.749 -1.12816 -1.75366 -0.00023577 0.00117675 4.2581 -0.963205 -2.16453 4.64133 
	894.042 614.645 893.77 493.359 -0.802799 -1.79314 0.000182051 0.00269701 1.82896 -0.64391 -2.10322 1.97268 
	602.116 302.303 601.015 227.248 -1.32861 -10.4952 0.000215779 0.00287148 19.6213 -1.33708 -10.4118 19.4644 

	X 
	-0.007764 -0.999956 0.005239 256.148590 
	0.000676 0.005234 0.999986 -511.077054 
	-0.999970 0.007768 0.000635 -2195.038552 
	0.000000 0.000000 0.000000 1.000000 

	Z 0
	0.998300 -0.058206 -0.002952 -15.311861 
	0.058232 0.998256 0.009658 37.452705 
	0.002384 -0.009813 0.999949 15.974169 
	0.000000 0.000000 0.000000 1.000000 

	Z 1
	0.998455 -0.054832 -0.009007 66.123654 
	0.054915 0.998449 0.009205 38.425469 
	0.008489 -0.009686 0.999917 -1.585840 
	0.000000 0.000000 0.000000 1.000000 

	Z 2
	0.998056 -0.061139 -0.012104 42.650376 
	0.061284 0.998048 0.012006 104.653622 
	0.011346 -0.012724 0.999855 42.838269 
	0.000000 0.000000 0.000000 1.000000 
 ```

B. Besides the directories for each method, there will be a directory titled `reconstructions`.  This has .ply files for the calibration pattern reconstructed using the calibration information from each method, including the ground truth (ideal.ply), and is sometimes useful for visualization purposes.  These model files can be viewed in a viewer such as [MeshLab](http://www.meshlab.net/).

C. `comparisons.txt`, shows the results according to the metrics we discuss in our paper, excepting the reconstruction accuracy metric.

D. `reconstruction_accuracy_error_comparisons.txt`, shows the individual results for the reconstruction accuracy metric.

````c++
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
			param_type = Quaternion;
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
			param_type = Quaternion;
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
			param_type = Quaternion;
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
			param_type = Quaternion;
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
			param_type = Quaternion;
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
			param_type = Quaternion;
			cost_type = rp2;
		} break;
````
