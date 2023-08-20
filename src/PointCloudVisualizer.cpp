#include "PointCloudVisualizer.h"

PointCloudVisualizer::PointCloudVisualizer(std::string name, Mode mode) : _name(name), _mode(mode), _framecount(0) {
    viewer = std::make_shared<pcl::visualization::PCLVisualizer>(_name);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);

    viewer->setWindowName(_name);
    double pos[3] = {215, 47, 585}; // cam position
    double view[3] = {193, 47, -1.1}; // center of view point
    double up[3] = {0, 1, 0}; // up position
    // double up[3] = {-0.0629535, 0.998014, 0.00225601}; // up position
    viewer->setCameraPosition(pos[0], pos[1], pos[2], view[0], view[1], view[2], up[0], up[1], up[2]);

    if (_mode == CAR) {
        // Define car dimensions (_length x _width x _height)
        _length = 10.0;
        _width = 4.0;
        _height = 3.0;

        // Compute the half dimensions for the cuboid
        double half_length = _length * 0.5;
        double half_width = _width * 0.5;
        double half_height = _height * 0.5;

        // As the car is initially at the origin, these are the bounds
        float x_min = -half_length;
        float x_max = half_length;
        float y_min = -half_width;
        float y_max = half_width;
        float z_min = -half_height;
        float z_max = half_height;

        // Add the car (cuboid) to the viewer with the bounds
        viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0.0, 1.0, 0.0, "car");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "car");  // Setting the color to green
    }
}

void PointCloudVisualizer::updateCarPose(const Eigen::Vector3d& t_w, const Eigen::Quaterniond& q_w) {

    // If a shape with the label "car" exists in the viewer, remove it.
    if (viewer->contains("car")) {
        viewer->removeShape("car");
    }

    // Calculate half dimensions of the car for ease of positioning.
    double half_length = _length * 0.5;
    double half_width = _width * 0.5;
    double half_height = _height * 0.5;

    // Define the 8 corners of the unrotated car (cuboid).
    Eigen::Vector3d p1(-half_length, -half_width, -half_height);
    Eigen::Vector3d p2(-half_length, -half_width, half_height);
    Eigen::Vector3d p3(-half_length, half_width, -half_height);
    Eigen::Vector3d p4(-half_length, half_width, half_height);
    Eigen::Vector3d p5(half_length, -half_width, -half_height);
    Eigen::Vector3d p6(half_length, -half_width, half_height);
    Eigen::Vector3d p7(half_length, half_width, -half_height);
    Eigen::Vector3d p8(half_length, half_width, half_height);

    // Rotate the corners using the quaternion and then translate them.
    p1 = q_w * p1 + t_w;
    p2 = q_w * p2 + t_w;
    p3 = q_w * p3 + t_w;
    p4 = q_w * p4 + t_w;
    p5 = q_w * p5 + t_w;
    p6 = q_w * p6 + t_w;
    p7 = q_w * p7 + t_w;
    p8 = q_w * p8 + t_w;

    // Now, compute the bounds of the rotated cuboid.
    float x_min = std::min({p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8[0]});
    float x_max = std::max({p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8[0]});
    float y_min = std::min({p1[1], p2[1], p3[1], p4[1], p5[1], p6[1], p7[1], p8[1]});
    float y_max = std::max({p1[1], p2[1], p3[1], p4[1], p5[1], p6[1], p7[1], p8[1]});
    float z_min = std::min({p1[2], p2[2], p3[2], p4[2], p5[2], p6[2], p7[2], p8[2]});
    float z_max = std::max({p1[2], p2[2], p3[2], p4[2], p5[2], p6[2], p7[2], p8[2]});

    // Add the rotated and translated cuboid to the visualizer using the computed bounds.
    viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0.0, 1.0, 0.0, "car");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "car");
}

// void PointCloudVisualizer::updateCarPose(const Eigen::Vector3d& t_w, const Eigen::Quaterniond& q_w) {

//     // Remove the current car if it exists
//     if (viewer->contains("car")) {
//         viewer->removeShape("car");
//     }

//     // Compute the half dimensions for the cuboid
//     double half_length = _length * 0.5;
//     double half_width = _width * 0.5;
//     double half_height = _height * 0.5;

//     // Compute bounds
//     float x_min = t_w[0] - half_length;
//     float x_max = t_w[0] + half_length;
//     float y_min = t_w[1] - half_width;
//     float y_max = t_w[1] + half_width;
//     float z_min = t_w[2] - half_height;
//     float z_max = t_w[2] + half_height;

//     // Use the addCube variant that accepts bounding values
//     viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 0.0, 1.0, 0.0, "car");
//     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "car");  // Setting the color to green
// }

void PointCloudVisualizer::updateTrajectory(const std::vector<Eigen::Vector3d> &t_w_path) {
    // Clear previous trajectory
    for (size_t i = 1; i < _trajectory.size(); i++) {
        std::string line_id = "line_" + std::to_string(i);
        if (viewer->contains(line_id)) {
            viewer->removeShape(line_id);
        }
    }
    
    // Update and display the new trajectory
    _trajectory = t_w_path; // Store the new trajectory data
    for (size_t i = 1; i < _trajectory.size(); i++) {
        std::string line_id = "line_" + std::to_string(i);
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(
            pcl::PointXYZ(_trajectory[i - 1](0), _trajectory[i - 1](1), _trajectory[i - 1](2)),
            pcl::PointXYZ(_trajectory[i](0), _trajectory[i](1), _trajectory[i](2)),
            1.0, 0.0, 0.0, // Red color
            line_id
        );
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_id);  // Set the line _width to 5
    }
}


void PointCloudVisualizer::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (!viewer->updatePointCloud(cloud, _name)) {
        viewer->addPointCloud(cloud, _name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, _name);
    }
}

void PointCloudVisualizer::updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color_handler(cloud, "intensity");
    
    if (!viewer->updatePointCloud<pcl::PointXYZI>(cloud, intensity_color_handler, _name)) {
        viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_color_handler, _name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, _name);
        
    }
}


void PointCloudVisualizer::removecloud() {
    viewer->removePointCloud(_name);
    // viewer->spinOnce(10);   
}
void PointCloudVisualizer::spinOnce() {
    viewer->spinOnce(400);
    _framecount++;
    std::string filename = "screenshot_" + std::to_string(_framecount) + ".png";
    viewer->saveScreenshot(filename);


    // viewer->getCameras(_cameras);

    // for (size_t i = 0; i < _cameras.size(); ++i) {
    //     std::cout << "Camera " << i+1 << " parameters:" << std::endl;
    //     std::cout << " - Position: [" << _cameras[i].pos[0] << ", " << _cameras[i].pos[1] << ", " << _cameras[i].pos[2] << "]" << std::endl;
    //     std::cout << " - Viewpoint: [" << _cameras[i].focal[0] << ", " << _cameras[i].focal[1] << ", " << _cameras[i].focal[2] << "]" << std::endl;
    //     std::cout << " - Up vector: [" << _cameras[i].view[0] << ", " << _cameras[i].view[1] << ", " << _cameras[i].view[2] << "]" << std::endl;
    // }

    

}

bool PointCloudVisualizer::wasStopped() {
    return viewer->wasStopped();
}
