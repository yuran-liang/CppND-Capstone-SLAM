// Rewrite of A-LOAM (https://github.com/HKUST-Aerial-Robotics/A-LOAM).
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// Modifications copyright (c) 2023, Yuran Liang
// Distributed under the BSD 3-Clause License, see LICENSE and NOTICE.

#ifndef SCAN_REGISTRATION_H
#define SCAN_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <cmath>
#include "helper.h"



using std::atan2;
using std::cos;
using std::sin;



class FeatureExtraction {

    public:
        FeatureExtraction();
        void registerCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        // Getters for each cloud type
        pcl::PointCloud<PointType>::Ptr getCornerPointsSharp();
        pcl::PointCloud<PointType>::Ptr getCornerPointsLessSharp();
        pcl::PointCloud<PointType>::Ptr getSurfPointsFlat();
        pcl::PointCloud<PointType>::Ptr getSurfPointsLessFlat();
        void clearCloud();

    private:
        pcl::PointCloud<PointType>::Ptr _cornerPointsSharp;
        pcl::PointCloud<PointType>::Ptr _cornerPointsLessSharp;
        pcl::PointCloud<PointType>::Ptr _surfPointsFlat;
        pcl::PointCloud<PointType>::Ptr _surfPointsLessFlat;


        void preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        

        void removeClosedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                float thres);

        const double _scanPeriod = 0.1;
        float _MINIMUM_RANGE = 5.5;
        int _N_SCANS = 64;
        float _cloudCurvature[400000];
        int _cloudSortInd[400000];
        int _cloudNeighborPicked[400000];
        int _cloudLabel[400000];

        bool _comp (int i,int j) { return (_cloudCurvature[i]<_cloudCurvature[j]); }
};


#endif // SCAN_REGISTRATION_H
