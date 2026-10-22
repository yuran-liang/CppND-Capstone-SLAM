# CPPND: Capstone Lidar SLAM

[![Build](https://github.com/yuran-liang/CppND-Capstone-SLAM/actions/workflows/build.yml/badge.svg)](https://github.com/yuran-liang/CppND-Capstone-SLAM/actions/workflows/build.yml)

This is a repo for the Capstone project in the [Udacity C++ Nanodegree Program](https://www.udacity.com/course/c-plus-plus-nanodegree--nd213). This project presents an implementation of the [LOAM (Lidar Odometry and Mapping in Real-time)](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf) algorithm.

<img src="images/demo.gif" width="756" height="491" />

## Dependencies for Running Locally
* cmake >= 3.10
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 3.81 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Ceres Solver >= 2.1
  * Follow [Ceres Installation](http://ceres-solver.org/installation.html).
* PCL >= 1.10
  * Follow [PCL Installation](https://pointclouds.org/downloads/).
  * `Very Important`: Due to a known issue in VTK 9.1.0's `vtkXRenderWindowInteractor` class, the program will encounter a `crash` resulting in a segmentation fault when running on `Ubuntu 22.04`. The issue was fixed from the PCL developers and reported to Ubuntu developers to potentially get a patched VTK version into 22.04, but it's unclear if/when that would happen. In this case, please `do not install the PCL library using apt`. Instead, please build it from the [master](https://github.com/PointCloudLibrary/pcl) branch. See the description of this behavior in issue [#5237](https://github.com/PointCloudLibrary/pcl/issues/5237). If you have already installed PCL via `apt`, please uninstall it carefully before building from master (e.g. using `sudo apt remove --purge libpcl-dev`).





Tested on Ubuntu 24.04 with the distribution packages `libpcl-dev` (PCL 1.14) and `libceres-dev` (Ceres 2.2).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run Lidar SLAM: `./LidarSLAM /path/to/your/pcd_folder`
5. Run Point Cloud Viewer: `./pcViewer /path/to/your/pcd_folder`

Without an argument, both programs read the bundled sample data from `../velodyne_pcd`.

## Sample Data

You can access the sample data in the [velodyne_pcd](./velodyne_pcd) directory. The frames are taken from sequence 00 of the [KITTI odometry dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) (Geiger et al., IJRR 2013), licensed under [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/).
## Description

LOAM is a state-of-the-art algorithm primarily designed for the purpose of real-time simultaneous localization and mapping (SLAM) using Lidar data. Developed with a keen focus on both efficiency and accuracy, LOAM demonstrates robust performance in varied environments, showcasing its capability to produce high-resolution maps while also estimating the sensor's motion in real-time.

This project represents both a rewrite and an advanced implementation of the [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) project, originally designed for ROS (Robot Operating System). I'm grateful to the original authors for providing a solid foundation for my implementation. Listed below are the primary differences and enhancements compared to the A-LOAM implementation:

1. Removed ROS dependencies for an independent system operation.
2. Incorporated Object-Oriented Programming principles, including abstraction, encapsulation, and others.
3. Introduced a Reader to fetch point cloud data from files.
4. Implemented a Point Cloud Viewer for reading and visualizing raw point clouds (refer to the main file: pc_viewer_main.cpp) utilizing PCL.
5. Added a Visualizer class to display the resulting point cloud from the SLAM algorithm, as well as to illustrate the pose and trajectory of the Lidar sensor.
6. Adopted smart pointers for efficient memory management.
7. Shifted from ROS messages to shared pointers for communication between Feature Extraction, Lidar Odometry, and Lidar Mapping, resulting in significant computational resource savings.

## Program Flow

### `main.cpp`
[main.cpp](src/main.cpp) is the entry point of the Lidar SLAM program. It initializes the core components like point cloud reader, feature extraction, laser odometry and odometry refinement modules. The main loop reads the point cloud frames from the data folder, processes them through each component, and visualizes the results.

The key steps are:
1. Initialize point cloud reader with data folder path.
2. Initialize feature extraction, laser odometry and odometry refinement class instances.
3. Start main loop to read and process frames.
   - Load next point cloud frame
   - Register current frame using feature extraction
   - Perform laser odometry calculation
   - Run odometry refinement with certain frequency
   - Visualize map point cloud
   - Update car pose and trajectory
   - Control frame rate
4. Exit when all frames processed or visualizer closed

### `FeatureExtraction.h` and `FeatureExtraction.cpp`
[FeatureExtraction.h](include/FeatureExtraction.h) and [FeatureExtraction.cpp](src/FeatureExtraction.cpp) define the FeatureExtraction class to extract features from lidar scans.

The header file declares the constructor, main process function, getters and private members. The source file implements the extraction logic in following steps:

1. Preprocess input cloud: filter, segment layers
2. Sort points and pick features based on curvature
3. Classify points into corner, surface types
4. Downsample surface points
5. Provide getters to access extracted clouds

By extracting geometric features like corners and planes from point cloud, the class provides key inputs for lidar odometry and mapping modules.

### `LaserOdom.h` and `LaserOdom.cpp`
[LaserOdom.h](include/LaserOdom.h) and [LaserOdom.cpp](src/LaserOdom.cpp) define the LaserOdometry class for lidar-based scan matching odometry estimation.

The source file implements:
1. Transforms points to start frame
2. Finds feature associations between frames
3. Sets up optimization problem
4. Calls Ceres solver to optimize
5. Updates current pose transform
6. Saves frame cloud as previous cloud

By optimizing edge and plane features, the LaserOdometry class performs continuous scan-to-scan matching to estimate lidar odometry.


### `OdoRefinement.h` and `OdoRefinement.cpp`
[OdoRefinement.h](include/OdoRefinement.h) and [OdoRefinement.cpp](src/OdoRefinement.cpp) define the OdoRefinement class to refine lidar odometry using mapping approach.

The source file implements:

1. Associates current points to map features
2. Sets up optimization problem based on associations
3. Solves to refine odometry pose
4. Computes transforms between map and odometry
5. Saves updated pose and transforms

By matching to historic map features, the OdoRefinement class refines the lidar odometry trajectory for higher accuracy.
 
### `PointCloudReader.h` and `PointCloudReader.cpp`
[PointCloudReader.h](include/PointCloudReader.h) and [PointCloudReader.cpp](src/PointCloudReader.cpp) define the PointCloudReader class to sequentially load lidar point cloud frames.

The implementation:

1. Constructor loads all pcd files in dataset directory
2. loadNext reads next pcd file into current frame
3. GetCloud returns shared ptr to current frame
4. ClearCloud clears existing point cloud data

By sequentially loading pcd files from dataset, PointCloudReader provides function to read lidar scans frame by frame.

### `PointCloudVisualizer.h` and `PointCloudVisualizer.cpp`
[PointCloudVisualizer.h](include/PointCloudVisualizer.h) and [PointCloudVisualizer.cpp](src/PointCloudVisualizer.cpp) define the PointCloudVisualizer class for visualization of point clouds and car model.

The implementation:

1. Constructor sets up PCL visualizer and adds car model if needed
2. updateCloud displays latest point cloud frame
3. updateCarPose removes and adds rotated car model
4. updateTrajectory displays smoothed trajectory
5. Utility functions like spinOnce and wasStopped

PointCloudVisualizer provides visualization capabilities for point cloud data and car position for lidar SLAM system.

### `helper.h`
[helper.h](include/helper.h) header file provides some common math helper functions.

The header is self-contained and provides common type aliases and angle conversion functions.

### `tic_toc.h`
[tic_toc.h](include/tic_toc.h) header file offers a utility class named TicToc for measuring elapsed time in milliseconds. It's designed to be lightweight and straightforward to use, relying on the C++ Standard Library's `chrono` for time measurements.

### `lidarFactor.hpp`
[lidarFactor.h](include/lidarFactor.hpp) header file defines several structures that encapsulate optimization factors related to Lidar measurements. These structures use the Ceres Solver library for optimization and Eigen for matrix operations, providing mechanisms to compute residuals for different constraints associated with Lidar data processing.

Features:
1. LidarEdgeFactor: Computes the edge constraint between the current point and two other points from a prior cloud.
2. LidarPlaneFactor: Enforces the plane constraint using the current point and three other points from the last cloud.
3. LidarPlaneNormFactor: Utilizes a point and the normal of a plane to compute the residual.
4. LidarDistanceFactor: Computes the Euclidean distance between the transformed current point and a given reference point.

### `pc_viewer_main.cpp`
[pc_viewer_main.cpp](src/pc_viewer_main.cpp) is the entry point of the Point Cloud Viewer
program. It is a simple application that reads and visualizes point cloud data using the PointCloudReader and PointCloudVisualizer utilities.

Features:
1. Flexible Directory Path: The directory from which to read point cloud data can be provided as a command-line argument. If not, a default path is used.

2. Rate-Controlled Visualization: The visualization loop has built-in control for frame duration, ensuring consistent visualization rate irrespective of processing time.

3. Graceful Exit: The application stops reading and visualizing when there are no more files left or if the visualization window is closed.

## License

This project is released under the [BSD 3-Clause License](LICENSE). It is a rewrite of [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM), which is based on LOAM by Ji Zhang (Carnegie Mellon University); see [NOTICE](NOTICE) for the upstream copyright notice and the sample data license.
