#include <cmath>

#include "LaserOdom.h"
#include "lidarFactor.hpp"
#include "tic_toc.h"
#include "helper.h"
#include <iostream>


using namespace std;

LaserOdometry::LaserOdometry() 
    : _frameCount(0),
      _skipFrameNum(5),
      _systemInited(false),
      _cornerPointsLastNum(0),
      _surfPointsFlatLastNum(0),
      _kdtreeCornerLast(std::make_shared<pcl::KdTreeFLANN<PointType>>()),
      _kdtreeSurfLast(std::make_shared<pcl::KdTreeFLANN<PointType>>()),
      _cornerPointsLast(std::make_shared<pcl::PointCloud<PointType>>()),
      _surfPointsFlatLast(std::make_shared<pcl::PointCloud<PointType>>()),
      _q_w_curr(Eigen::Quaterniond::Identity()),
      _t_w_curr(0.0, 0.0, 0.0),
      _para_q{0.0, 0.0, 0.0, 1.0},
      _para_t{0.0, 0.0, 0.0},
      _q_last_curr(Eigen::Map<Eigen::Quaterniond>(_para_q)),
      _t_last_curr(Eigen::Map<Eigen::Vector3d>(_para_t))
{}

void LaserOdometry::transformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, _q_last_curr);
    Eigen::Vector3d t_point_last = s * _t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();

    po->intensity = pi->intensity;
    
}

void LaserOdometry::savestate(pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp, pcl::PointCloud<PointType>::Ptr surfPointsLessFlat)
{
    _q_w_path.push_back(_q_w_curr);
    _t_w_path.push_back(_t_w_curr);

    *_cornerPointsLast = *cornerPointsLessSharp;
    *_surfPointsFlatLast = *surfPointsLessFlat;
   
    // pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    // cornerPointsLessSharp = _cornerPointsLast;
    // _cornerPointsLast = laserCloudTemp;

    // laserCloudTemp = surfPointsLessFlat;
    // surfPointsLessFlat = _surfPointsFlatLast;
    // _surfPointsFlatLast = laserCloudTemp;

    _cornerPointsLastNum = _cornerPointsLast->points.size();
    _surfPointsFlatLastNum = _surfPointsFlatLast->points.size();


    _kdtreeCornerLast->setInputCloud(_cornerPointsLast);
    _kdtreeSurfLast->setInputCloud(_surfPointsFlatLast);
}


void LaserOdometry::process(std::shared_ptr<FeatureExtraction> scanRegister) {
    // cout << "Mapping " << 10 / skipFrameNum <<  " Hz \n";
    TicToc t_whole;

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp = scanRegister->getCornerPointsSharp();
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp = scanRegister->getCornerPointsLessSharp();
    pcl::PointCloud<PointType>::Ptr surfPointsFlat = scanRegister->getSurfPointsFlat();
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat = scanRegister->getSurfPointsLessFlat();
        

    if (!_systemInited) {
        _systemInited = true;
    }
    else {
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum = surfPointsFlat->points.size();
        TicToc t_opt;
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
        {
            int corner_correspondence = 0;
            int plane_correspondence = 0;

            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            
            ceres::Manifold *q_manifold = new ceres::EigenQuaternionManifold();

            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(_para_q, 4, q_manifold);
            problem.AddParameterBlock(_para_t, 3);

            // //ceres::LossFunction *loss_function = NULL;
            // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            // ceres::LocalParameterization *q_parameterization =
            //     new ceres::EigenQuaternionParameterization();
            // ceres::Problem::Options problem_options;

            // ceres::Problem problem(problem_options);
            // problem.AddParameterBlock(_para_q, 4, q_parameterization);
            // problem.AddParameterBlock(_para_t, 3);

            pcl::PointXYZI pointSel;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            TicToc t_data;
            // find correspondence for corner features
            for (int i = 0; i < cornerPointsSharpNum; ++i)
            {
                transformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                _kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                int closestPointInd = -1, minPointInd2 = -1;
                if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                {
                    closestPointInd = pointSearchInd[0];
                    int closestPointScanID = int(_cornerPointsLast->points[closestPointInd].intensity);

                    double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                    // search in the direction of increasing scan line
                    for (int j = closestPointInd + 1; j < (int)_cornerPointsLast->points.size(); ++j)
                    {
                        // if in the same scan line, continue
                        if (int(_cornerPointsLast->points[j].intensity) <= closestPointScanID)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(_cornerPointsLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                            break;

                        double pointSqDis = (_cornerPointsLast->points[j].x - pointSel.x) *
                                                (_cornerPointsLast->points[j].x - pointSel.x) +
                                            (_cornerPointsLast->points[j].y - pointSel.y) *
                                                (_cornerPointsLast->points[j].y - pointSel.y) +
                                            (_cornerPointsLast->points[j].z - pointSel.z) *
                                                (_cornerPointsLast->points[j].z - pointSel.z);

                        if (pointSqDis < minPointSqDis2)
                        {
                            // find nearer point
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closestPointInd - 1; j >= 0; --j)
                    {
                        // if in the same scan line, continue
                        if (int(_cornerPointsLast->points[j].intensity) >= closestPointScanID)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(_cornerPointsLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                            break;

                        double pointSqDis = (_cornerPointsLast->points[j].x - pointSel.x) *
                                                (_cornerPointsLast->points[j].x - pointSel.x) +
                                            (_cornerPointsLast->points[j].y - pointSel.y) *
                                                (_cornerPointsLast->points[j].y - pointSel.y) +
                                            (_cornerPointsLast->points[j].z - pointSel.z) *
                                                (_cornerPointsLast->points[j].z - pointSel.z);

                        if (pointSqDis < minPointSqDis2)
                        {
                            // find nearer point
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }
                if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                {
                    Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                cornerPointsSharp->points[i].y,
                                                cornerPointsSharp->points[i].z);
                    Eigen::Vector3d last_point_a(_cornerPointsLast->points[closestPointInd].x,
                                                    _cornerPointsLast->points[closestPointInd].y,
                                                    _cornerPointsLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(_cornerPointsLast->points[minPointInd2].x,
                                                    _cornerPointsLast->points[minPointInd2].y,
                                                    _cornerPointsLast->points[minPointInd2].z);

                    double s;
                    if (DISTORTION)
                        s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                    else
                        s = 1.0;
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                    problem.AddResidualBlock(cost_function, loss_function, _para_q, _para_t);
                    corner_correspondence++;
                }
            }

            // find correspondence for plane features
            for (int i = 0; i < surfPointsFlatNum; ++i)
            {
                transformToStart(&(surfPointsFlat->points[i]), &pointSel);
                _kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                {
                    closestPointInd = pointSearchInd[0];

                    // get closest point's scan ID
                    int closestPointScanID = int(_surfPointsFlatLast->points[closestPointInd].intensity);
                    double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                    // search in the direction of increasing scan line
                    for (int j = closestPointInd + 1; j < (int)_surfPointsFlatLast->points.size(); ++j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(_surfPointsFlatLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                            break;

                        double pointSqDis = (_surfPointsFlatLast->points[j].x - pointSel.x) *
                                                (_surfPointsFlatLast->points[j].x - pointSel.x) +
                                            (_surfPointsFlatLast->points[j].y - pointSel.y) *
                                                (_surfPointsFlatLast->points[j].y - pointSel.y) +
                                            (_surfPointsFlatLast->points[j].z - pointSel.z) *
                                                (_surfPointsFlatLast->points[j].z - pointSel.z);

                        // if in the same or lower scan line
                        if (int(_surfPointsFlatLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        // if in the higher scan line
                        else if (int(_surfPointsFlatLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                        {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closestPointInd - 1; j >= 0; --j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(_surfPointsFlatLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                            break;

                        double pointSqDis = (_surfPointsFlatLast->points[j].x - pointSel.x) *
                                                (_surfPointsFlatLast->points[j].x - pointSel.x) +
                                            (_surfPointsFlatLast->points[j].y - pointSel.y) *
                                                (_surfPointsFlatLast->points[j].y - pointSel.y) +
                                            (_surfPointsFlatLast->points[j].z - pointSel.z) *
                                                (_surfPointsFlatLast->points[j].z - pointSel.z);

                        // if in the same or higher scan line
                        if (int(_surfPointsFlatLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                        {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                        else if (int(_surfPointsFlatLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                        {
                            // find nearer point
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }

                    if (minPointInd2 >= 0 && minPointInd3 >= 0)
                    {

                        Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                    surfPointsFlat->points[i].y,
                                                    surfPointsFlat->points[i].z);
                        Eigen::Vector3d last_point_a(_surfPointsFlatLast->points[closestPointInd].x,
                                                        _surfPointsFlatLast->points[closestPointInd].y,
                                                        _surfPointsFlatLast->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(_surfPointsFlatLast->points[minPointInd2].x,
                                                        _surfPointsFlatLast->points[minPointInd2].y,
                                                        _surfPointsFlatLast->points[minPointInd2].z);
                        Eigen::Vector3d last_point_c(_surfPointsFlatLast->points[minPointInd3].x,
                                                        _surfPointsFlatLast->points[minPointInd3].y,
                                                        _surfPointsFlatLast->points[minPointInd3].z);

                        double s;
                        if (DISTORTION)
                            s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                        else
                            s = 1.0;
                        ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                        problem.AddResidualBlock(cost_function, loss_function, _para_q, _para_t);
                        plane_correspondence++;
                    }
                }
            }

            // cout << "data association time " << t_data.toc() << " ms \n";

            if ((corner_correspondence + plane_correspondence) < 10)
            {   
                cout << "less correspondence! *************************************************\n";
            }

            TicToc t_solver;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // cout << "solver time " << t_solver.toc() << " ms \n";
        }
        cout << "optimization twice time " << t_opt.toc() <<  " ms \n";
        _t_w_curr = _t_w_curr + _q_w_curr * _t_last_curr;
        _q_w_curr = _q_w_curr * _q_last_curr;
        // cout << "Current delta position x = " << _t_last_curr[0] << " m    y = " << _t_last_curr[1] << " m    z = " << _t_last_curr[2]<<  " m \n";
        // cout << "Current position x = " << _t_w_curr[0] << " m    y = " << _t_w_curr[1] << " m    z = " << _t_w_curr[2]<<  " m \n";
    }
    
    // update states
    this->savestate(cornerPointsLessSharp, surfPointsLessFlat);
    _frameCount++;

    cout << "whole laserOdometry time " << t_whole.toc() <<  "ms \n \n";
    if(t_whole.toc() > 100) {
        cout << "[WARNING] Odometry process over 100ms" << endl;
    }

  


}


// Getter
Eigen::Vector3d LaserOdometry::GetTwCurr() const {return _t_w_curr;}
Eigen::Quaterniond LaserOdometry::GetQwCurr() const {return _q_w_curr;}
pcl::PointCloud<PointType>::Ptr LaserOdometry::GetCornerPointsLast() {return _cornerPointsLast;}
pcl::PointCloud<PointType>::Ptr LaserOdometry::GetSurfPointsFlatLast() {return _surfPointsFlatLast;}
int LaserOdometry::GetFrameCount() {return _frameCount;}
int LaserOdometry::GetSkipFrameNum() {return _skipFrameNum;}
void LaserOdometry::SetFrameCount(int count){_frameCount = count;}
std::vector<Eigen::Quaterniond> LaserOdometry::GetQwPath() const {return _q_w_path;}
std::vector<Eigen::Vector3d> LaserOdometry::GetTwPath() const {return _t_w_path;}