#ifndef POINTCLOUDVISUALIZER_H
#define POINTCLOUDVISUALIZER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class PointCloudVisualizer {
public:
    enum Mode {
        CAR,
        POINTCLOUD
    };
    PointCloudVisualizer(std::string name, Mode mode = POINTCLOUD);
    void removecloud();
    // Update point cloud
    void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    // Update car's position and orientation
    void updateCarPose(const Eigen::Vector3d &t_w, const Eigen::Quaterniond &q_w);

    // Update and display the trajectory based on the latest path
    void updateTrajectory(const std::vector<Eigen::Vector3d> &t_w_path);
    
    void spinOnce();
    // void show(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    bool wasStopped();


private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::string _name;
    int _framecount;
    std::vector<pcl::visualization::Camera> _cameras;
    Mode _mode;
    std::vector<Eigen::Vector3d> _trajectory;
    // car dimensions (_length x _width x _height)

    double _length;
    double _width;
    double _height;

};

#endif