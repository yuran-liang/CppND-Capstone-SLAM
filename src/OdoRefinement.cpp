// Rewrite of A-LOAM (https://github.com/HKUST-Aerial-Robotics/A-LOAM).
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// Modifications copyright (c) 2023, Yuran Liang
// Distributed under the BSD 3-Clause License, see LICENSE and NOTICE.

#include "OdoRefinement.h"
#include <iostream>
#include "tic_toc.h"
#include <cmath>
#include "lidarFactor.hpp"
#include <cstdio>

using namespace std;

OdoRefinement::OdoRefinement() 
    : _frameCount(0),
      _laserCloudCenWidth(10),
      _laserCloudCenHeight(10),
      _laserCloudCenDepth(5),
      _laserCloudSurround(std::make_shared<pcl::PointCloud<PointType>>()),
      _laserCloudCornerFromMap(std::make_shared<pcl::PointCloud<PointType>>()),
      _laserCloudSurfFromMap(std::make_shared<pcl::PointCloud<PointType>>()),
      _kdtreeCornerFromMap(std::make_shared<pcl::KdTreeFLANN<PointType>>()),
      _kdtreeSurfFromMap(std::make_shared<pcl::KdTreeFLANN<PointType>>()),
      _parameters{0, 0, 0, 1, 0, 0, 0},
      _q_w_curr(Eigen::Map<Eigen::Quaterniond>(_parameters)),
      _t_w_curr(Eigen::Map<Eigen::Vector3d>(_parameters + 4)),
      _q_wmap_wodom(Eigen::Quaterniond::Identity()),
      _t_wmap_wodom(0.0, 0.0, 0.0),
      _q_wodom_curr(Eigen::Map<Eigen::Quaterniond>(_parameters)),
      _t_wodom_curr(0.0, 0.0, 0.0),
      _lineRes(0.4),
      _planeRes(0.8),
      _q_wmap_wodom_high_freq(Eigen::Quaterniond::Identity()),
      _t_wmap_wodom_high_freq(0.0, 0.0, 0.0)
{
    
    _downSizeFilterCorner.setLeafSize(_lineRes, _lineRes,_lineRes);
	_downSizeFilterSurf.setLeafSize(_planeRes, _planeRes, _planeRes);

    for (int i = 0; i < _laserCloudNum; i++)
	{
		_laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		_laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}
}

// set initial guess
void OdoRefinement::transformAssociateToMap()
{
	_q_w_curr = _q_wmap_wodom * _q_wodom_curr;
	_t_w_curr = _q_wmap_wodom * _t_wodom_curr + _t_wmap_wodom;
}

void OdoRefinement::transformUpdate()
{
	_q_wmap_wodom = _q_w_curr * _q_wodom_curr.inverse();
	_t_wmap_wodom = _t_w_curr - _q_wmap_wodom * _t_wodom_curr;
}

void OdoRefinement::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = _q_w_curr * point_curr + _t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

void OdoRefinement::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = _q_w_curr.inverse() * (point_w - _t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}

void OdoRefinement::updateodomhighfreq(Eigen::Vector3d t_curr, Eigen::Quaterniond q_curr){
    // convert odo frame to map frame
    _q_wmap_wodom_high_freq = _q_wmap_wodom * q_curr;
	_t_wmap_wodom_high_freq = _q_wmap_wodom * t_curr + _t_wmap_wodom; 
}


void OdoRefinement::process(std::shared_ptr<LaserOdometry> laserodom) {
    TicToc t_whole;

    // Save odo state to current state
    _q_wodom_curr = laserodom->GetQwCurr();
    _t_wodom_curr = laserodom->GetTwCurr();

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast = laserodom->GetCornerPointsLast();
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast = laserodom->GetSurfPointsFlatLast();

    // convert odo frame to map frame as initial guess
    transformAssociateToMap();

    // Generate grid map
    TicToc t_shift;
    int centerCubeI = int((_t_w_curr.x() + 25.0) / 50.0) + _laserCloudCenWidth;
    int centerCubeJ = int((_t_w_curr.y() + 25.0) / 50.0) + _laserCloudCenHeight;
    int centerCubeK = int((_t_w_curr.z() + 25.0) / 50.0) + _laserCloudCenDepth;

    if (_t_w_curr.x() + 25.0 < 0)
        centerCubeI--;
    if (_t_w_curr.y() + 25.0 < 0)
        centerCubeJ--;
    if (_t_w_curr.z() + 25.0 < 0)
        centerCubeK--;

    while (centerCubeI < 3)
    {
        for (int j = 0; j < _laserCloudHeight; j++)
        {
            for (int k = 0; k < _laserCloudDepth; k++)
            { 
                int i = _laserCloudWidth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k]; 
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                for (; i >= 1; i--)
                {
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudCornerArray[i - 1 + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudSurfArray[i - 1 + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                }
                _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI++;
        _laserCloudCenWidth++;
    }

    while (centerCubeI >= _laserCloudWidth - 3)
    { 
        for (int j = 0; j < _laserCloudHeight; j++)
        {
            for (int k = 0; k < _laserCloudDepth; k++)
            {
                int i = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                for (; i < _laserCloudWidth - 1; i++)
                {
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudCornerArray[i + 1 + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudSurfArray[i + 1 + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                }
                _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        _laserCloudCenWidth--;
    }

    while (centerCubeJ < 3)
    {
        for (int i = 0; i < _laserCloudWidth; i++)
        {
            for (int k = 0; k < _laserCloudDepth; k++)
            {
                int j = _laserCloudHeight - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                for (; j >= 1; j--)
                {
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudCornerArray[i + _laserCloudWidth * (j - 1) + _laserCloudWidth * _laserCloudHeight * k];
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudSurfArray[i + _laserCloudWidth * (j - 1) + _laserCloudWidth * _laserCloudHeight * k];
                }
                _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        _laserCloudCenHeight++;
    }

    while (centerCubeJ >= _laserCloudHeight - 3)
    {
        for (int i = 0; i < _laserCloudWidth; i++)
        {
            for (int k = 0; k < _laserCloudDepth; k++)
            {
                int j = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                for (; j < _laserCloudHeight - 1; j++)
                {
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudCornerArray[i + _laserCloudWidth * (j + 1) + _laserCloudWidth * _laserCloudHeight * k];
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudSurfArray[i + _laserCloudWidth * (j + 1) + _laserCloudWidth * _laserCloudHeight * k];
                }
                _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        _laserCloudCenHeight--;
    }

    while (centerCubeK < 3)
    {
        for (int i = 0; i < _laserCloudWidth; i++)
        {
            for (int j = 0; j < _laserCloudHeight; j++)
            {
                int k = _laserCloudDepth - 1;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                for (; k >= 1; k--)
                {
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * (k - 1)];
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * (k - 1)];
                }
                _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        _laserCloudCenDepth++;
    }

    while (centerCubeK >= _laserCloudDepth - 3)
    {
        for (int i = 0; i < _laserCloudWidth; i++)
        {
            for (int j = 0; j < _laserCloudHeight; j++)
            {
                int k = 0;
                pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k];
                for (; k < _laserCloudDepth - 1; k++)
                {
                    _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * (k + 1)];
                    _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                        _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * (k + 1)];
                }
                _laserCloudCornerArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeCornerPointer;
                _laserCloudSurfArray[i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k] =
                    laserCloudCubeSurfPointer;
                laserCloudCubeCornerPointer->clear();
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK--;
        _laserCloudCenDepth--;
    }

    int laserCloudValidNum = 0;
	int laserCloudSurroundNum = 0;

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
        {
            for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
            {
                if (i >= 0 && i < _laserCloudWidth &&
                    j >= 0 && j < _laserCloudHeight &&
                    k >= 0 && k < _laserCloudDepth)
                { 
                    _laserCloudValidInd[laserCloudValidNum] = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
                    laserCloudValidNum++;
                    _laserCloudSurroundInd[laserCloudSurroundNum] = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
                    laserCloudSurroundNum++;
                }
            }
        }
    }

    _laserCloudCornerFromMap->clear();
    _laserCloudSurfFromMap->clear();
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        *_laserCloudCornerFromMap += *_laserCloudCornerArray[_laserCloudValidInd[i]];
        *_laserCloudSurfFromMap += *_laserCloudSurfArray[_laserCloudValidInd[i]];
    }
    int laserCloudCornerFromMapNum = _laserCloudCornerFromMap->points.size();
    int laserCloudSurfFromMapNum = _laserCloudSurfFromMap->points.size();


    pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
    _downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    _downSizeFilterCorner.filter(*laserCloudCornerStack);
    int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

    pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
    _downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    _downSizeFilterSurf.filter(*laserCloudSurfStack);
    int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

    // printf("map prepare time %f ms\n", t_shift.toc());
    // printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
    if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
    {
        TicToc t_opt;
        TicToc t_tree;
        _kdtreeCornerFromMap->setInputCloud(_laserCloudCornerFromMap);
        _kdtreeSurfFromMap->setInputCloud(_laserCloudSurfFromMap);
        // printf("build tree time %f ms \n", t_tree.toc());

        for (int iterCount = 0; iterCount < 2; iterCount++)
        {
            //ceres::LossFunction *loss_function = NULL;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Manifold *q_manifold = new ceres::EigenQuaternionManifold();
            ceres::Problem::Options problem_options;

            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(_parameters, 4, q_manifold);
            problem.AddParameterBlock(_parameters + 4, 3);

            TicToc t_data;
            int corner_num = 0;

            for (int i = 0; i < laserCloudCornerStackNum; i++)
            {
                _pointOri = laserCloudCornerStack->points[i];
                //double sqrtDis = _pointOri.x * _pointOri.x + _pointOri.y * _pointOri.y + _pointOri.z * _pointOri.z;
                pointAssociateToMap(&_pointOri, &_pointSel);
                _kdtreeCornerFromMap->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis); 

                if (_pointSearchSqDis[4] < 1.0)
                { 
                    std::vector<Eigen::Vector3d> nearCorners;
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(_laserCloudCornerFromMap->points[_pointSearchInd[j]].x,
                                            _laserCloudCornerFromMap->points[_pointSearchInd[j]].y,
                                            _laserCloudCornerFromMap->points[_pointSearchInd[j]].z);
                        center = center + tmp;
                        nearCorners.push_back(tmp);
                    }
                    center = center / 5.0;

                    Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                    }

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                    // if is indeed line feature
                    // note Eigen library sort eigenvalues in increasing order
                    Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                    Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    { 
                        Eigen::Vector3d point_on_line = center;
                        Eigen::Vector3d point_a, point_b;
                        point_a = 0.1 * unit_direction + point_on_line;
                        point_b = -0.1 * unit_direction + point_on_line;

                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                        problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
                        corner_num++;	
                    }							
                }
            }

            int surf_num = 0;
            for (int i = 0; i < laserCloudSurfStackNum; i++)
            {
                _pointOri = laserCloudSurfStack->points[i];
                //double sqrtDis = _pointOri.x * _pointOri.x + _pointOri.y * _pointOri.y + _pointOri.z * _pointOri.z;
                pointAssociateToMap(&_pointOri, &_pointSel);
                _kdtreeSurfFromMap->nearestKSearch(_pointSel, 5, _pointSearchInd, _pointSearchSqDis);

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (_pointSearchSqDis[4] < 1.0)
                {
                    
                    for (int j = 0; j < 5; j++)
                    {
                        matA0(j, 0) = _laserCloudSurfFromMap->points[_pointSearchInd[j]].x;
                        matA0(j, 1) = _laserCloudSurfFromMap->points[_pointSearchInd[j]].y;
                        matA0(j, 2) = _laserCloudSurfFromMap->points[_pointSearchInd[j]].z;
                        //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++)
                    {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) * _laserCloudSurfFromMap->points[_pointSearchInd[j]].x +
                                    norm(1) * _laserCloudSurfFromMap->points[_pointSearchInd[j]].y +
                                    norm(2) * _laserCloudSurfFromMap->points[_pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                        {
                            planeValid = false;
                            break;
                        }
                    }
                    Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
                    if (planeValid)
                    {
                        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                        problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
                        surf_num++;
                    }
                }
                /*
                else if(_pointSearchSqDis[4] < 0.01 * sqrtDis)
                {
                    Eigen::Vector3d center(0, 0, 0);
                    for (int j = 0; j < 5; j++)
                    {
                        Eigen::Vector3d tmp(_laserCloudSurfFromMap->points[_pointSearchInd[j]].x,
                                            _laserCloudSurfFromMap->points[_pointSearchInd[j]].y,
                                            _laserCloudSurfFromMap->points[_pointSearchInd[j]].z);
                        center = center + tmp;
                    }
                    center = center / 5.0;	
                    Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
                    ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                    problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
                }
                */
            }

            //printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
            //printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

            // printf("mapping data assosiation time %f ms \n", t_data.toc());

            TicToc t_solver;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            // printf("mapping solver time %f ms \n", t_solver.toc());

            //printf("time %f \n", timeLaserOdometry);
            //printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
            //printf("result q %f %f %f %f result t %f %f %f\n", _parameters[3], _parameters[0], _parameters[1], _parameters[2],
            //	   _parameters[4], _parameters[5], _parameters[6]);
        }
        printf("mapping optimization time %f \n", t_opt.toc());
    }
    else
    {
        cout << "[WARNING] Time Map corner and surf num are not enough" << endl;
    }
    transformUpdate();

    TicToc t_add;
    for (int i = 0; i < laserCloudCornerStackNum; i++)
    {
        pointAssociateToMap(&laserCloudCornerStack->points[i], &_pointSel);

        int cubeI = int((_pointSel.x + 25.0) / 50.0) + _laserCloudCenWidth;
        int cubeJ = int((_pointSel.y + 25.0) / 50.0) + _laserCloudCenHeight;
        int cubeK = int((_pointSel.z + 25.0) / 50.0) + _laserCloudCenDepth;

        if (_pointSel.x + 25.0 < 0)
            cubeI--;
        if (_pointSel.y + 25.0 < 0)
            cubeJ--;
        if (_pointSel.z + 25.0 < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < _laserCloudWidth &&
            cubeJ >= 0 && cubeJ < _laserCloudHeight &&
            cubeK >= 0 && cubeK < _laserCloudDepth)
        {
            int cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
            _laserCloudCornerArray[cubeInd]->push_back(_pointSel);
        }
    }

    for (int i = 0; i < laserCloudSurfStackNum; i++)
    {
        pointAssociateToMap(&laserCloudSurfStack->points[i], &_pointSel);

        int cubeI = int((_pointSel.x + 25.0) / 50.0) + _laserCloudCenWidth;
        int cubeJ = int((_pointSel.y + 25.0) / 50.0) + _laserCloudCenHeight;
        int cubeK = int((_pointSel.z + 25.0) / 50.0) + _laserCloudCenDepth;

        if (_pointSel.x + 25.0 < 0)
            cubeI--;
        if (_pointSel.y + 25.0 < 0)
            cubeJ--;
        if (_pointSel.z + 25.0 < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < _laserCloudWidth &&
            cubeJ >= 0 && cubeJ < _laserCloudHeight &&
            cubeK >= 0 && cubeK < _laserCloudDepth)
        {
            int cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
            _laserCloudSurfArray[cubeInd]->push_back(_pointSel);
        }
    }
    // printf("add points time %f ms\n", t_add.toc());

    _q_w_path.push_back(_q_w_curr);
    _t_w_path.push_back(_t_w_curr);

    
    TicToc t_filter;
    for (int i = 0; i < laserCloudValidNum; i++)
    {
        int ind = _laserCloudValidInd[i];

        pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
        _downSizeFilterCorner.setInputCloud(_laserCloudCornerArray[ind]);
        _downSizeFilterCorner.filter(*tmpCorner);
        _laserCloudCornerArray[ind] = tmpCorner;

        pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
        _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
        _downSizeFilterSurf.filter(*tmpSurf);
        _laserCloudSurfArray[ind] = tmpSurf;
    }
    // printf("filter time %f ms \n", t_filter.toc());

    //save surround map for every 5 frame
    if (_frameCount % 5 == 0)
    {
        _laserCloudSurround->clear();
        for (int i = 0; i < laserCloudSurroundNum; i++)
        {
            int ind = _laserCloudSurroundInd[i];
            *_laserCloudSurround += *_laserCloudCornerArray[ind];
            *_laserCloudSurround += *_laserCloudSurfArray[ind];
        }

    }




}


pcl::PointCloud<PointType>::Ptr OdoRefinement::GetCloudSurround(){return _laserCloudSurround;}
Eigen::Vector3d OdoRefinement::GetTwCurr() const {return _t_w_curr;}
Eigen::Quaterniond OdoRefinement::GetQwCurr() const {return _q_w_curr;}
std::vector<Eigen::Quaterniond> OdoRefinement::GetQwPath() const {return _q_w_path;}
std::vector<Eigen::Vector3d> OdoRefinement::GetTwPath() const {return _t_w_path;}
Eigen::Vector3d OdoRefinement::GetTwCurrHighFreq() const {return _t_wmap_wodom_high_freq;}
Eigen::Quaterniond OdoRefinement::GetQwCurrHighFreq() const {return _q_wmap_wodom_high_freq;}

