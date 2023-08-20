#ifndef LASERODOM_H
#define LASERODOM_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>
#include "helper.h"
#include "FeatureExtraction.h"


class LaserOdometry{
    // constant definition
    static constexpr double SCAN_PERIOD = 0.1;
    static constexpr double DISTANCE_SQ_THRESHOLD = 25; 
    static constexpr double NEARBY_SCAN = 2.5;
    static constexpr bool DISTORTION = false;

public:

    LaserOdometry();

    void process(std::shared_ptr<FeatureExtraction> scanRegister);

    //Getter
    Eigen::Vector3d GetTwCurr() const;
    Eigen::Quaterniond GetQwCurr() const;
    pcl::PointCloud<PointType>::Ptr GetCornerPointsLast();
    pcl::PointCloud<PointType>::Ptr GetSurfPointsFlatLast();
    int GetFrameCount();
    int GetSkipFrameNum();
    void SetFrameCount(int count);
    std::vector<Eigen::Quaterniond> GetQwPath() const;
    std::vector<Eigen::Vector3d> GetTwPath() const;


private:
    void transformToStart(PointType const *const pi, PointType *const po);
    void savestate(pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp, pcl::PointCloud<PointType>::Ptr surfPointsLessFlat);
    int _frameCount;

    // System status
    int _skipFrameNum; 
    bool _systemInited;


    // KD-tree objects 
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeCornerLast;
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeSurfLast;

    // Point cloud objects
    pcl::PointCloud<PointType>::Ptr _cornerPointsLast;
    pcl::PointCloud<PointType>::Ptr _surfPointsFlatLast;

    int _cornerPointsLastNum;
    int _surfPointsFlatLastNum;

    // Transformation from current frame to world frame
    Eigen::Quaterniond _q_w_curr;
    Eigen::Vector3d _t_w_curr;
    std::vector<Eigen::Quaterniond> _q_w_path;
    std::vector<Eigen::Vector3d> _t_w_path;

    
    // Intermediate variables
    // q_curr_last(x, y, z, w), t_curr_last
    double _para_q[4];
    double _para_t[3];
    Eigen::Map<Eigen::Quaterniond> _q_last_curr;
    Eigen::Map<Eigen::Vector3d> _t_last_curr;


};

#endif