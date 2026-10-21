// Rewrite of A-LOAM (https://github.com/HKUST-Aerial-Robotics/A-LOAM).
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// Modifications copyright (c) 2023, Yuran Liang
// Distributed under the BSD 3-Clause License, see LICENSE and NOTICE.

#include "FeatureExtraction.h"
#include "tic_toc.h"
#include <iostream>
#include <cassert>
#include <pcl/filters/voxel_grid.h>


using namespace std;

FeatureExtraction::FeatureExtraction() 
    : _cornerPointsSharp(new pcl::PointCloud<PointType>()),
      _cornerPointsLessSharp(new pcl::PointCloud<PointType>()),
      _surfPointsFlat(new pcl::PointCloud<PointType>()),
      _surfPointsLessFlat(new pcl::PointCloud<PointType>()) {}


// Abstract implementation details from their interfaces
void FeatureExtraction::preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(_N_SCANS, 0);
    std::vector<int> scanEndInd(_N_SCANS, 0);

    // 1. Preprocessing
    // Remove NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, indices);
    // Remove points closed to the sensor
    removeClosedPointCloud(cloud, _MINIMUM_RANGE);

    // 2. Calculate Layer
    int cloudSize = cloud->points.size();
    float startOri = -atan2(cloud->points[0].y, cloud->points[0].x);
    float endOri = -atan2(cloud->points[cloudSize - 1].y,
                          cloud->points[cloudSize - 1].x) +
                   2 * M_PI;
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudScans(_N_SCANS);
    // Initialize each pointer to a new point cloud object
    for (auto& ptr : laserCloudScans) {
        ptr = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    for (int i = 0; i < cloudSize; i++)
    {
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (_N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (_N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (_N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (_N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (_N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = _N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {   cout << "wrong scan number\n";
            assert(false);
        }

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + _scanPeriod * relTime;
        laserCloudScans[scanID]->points.push_back(point);
    }

    cloudSize = count;
    // cout << "points size " << "\n" << cloudSize;

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < _N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += *laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    // cout << "prepare time " << t_prepare.toc() << "\n";

    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        _cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        _cloudSortInd[i] = i;
        _cloudNeighborPicked[i] = 0;
        _cloudLabel[i] = 0;
    }


    TicToc t_pts;

    _cornerPointsSharp->clear();
    _cornerPointsLessSharp->clear();
    _surfPointsFlat->clear();
    _surfPointsLessFlat->clear();

    float t_q_sort = 0;

    for (int i = 0; i < _N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort(_cloudSortInd + sp, _cloudSortInd + ep + 1, 
            [this](const auto& a, const auto& b) {
                return this->_comp(a, b);
            });
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = _cloudSortInd[k]; 

                if (_cloudNeighborPicked[ind] == 0 &&
                    _cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {                        
                        _cloudLabel[ind] = 2;
                        _cornerPointsSharp->push_back(laserCloud->points[ind]);
                        _cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {                        
                        _cloudLabel[ind] = 1; 
                        _cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    _cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        _cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        _cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = _cloudSortInd[k];

                if (_cloudNeighborPicked[ind] == 0 &&
                    _cloudCurvature[ind] < 0.1)
                {

                    _cloudLabel[ind] = -1; 
                    _surfPointsFlat->push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    _cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        _cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        _cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (_cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        *_surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    // cout << "sort q time " << t_q_sort << endl;
    // cout << "seperate points time " << t_pts.toc() << endl;

    // cout << "scan registration time " << t_whole.toc() << " ms *************\n";

    if(t_whole.toc() > 100) {
            cout << "[WARNING] scan registration process over 100ms" << endl;
    }
    
}

void FeatureExtraction::removeClosedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inout,
                                              float thres) {
    cloud_inout->points.erase(
    std::remove_if(cloud_inout->points.begin(), cloud_inout->points.end(),
                   [thres](const pcl::PointXYZ& point) {
                       return point.x * point.x + point.y * point.y + point.z * point.z < thres * thres;
                   }),
    cloud_inout->points.end());

}

void FeatureExtraction::registerCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    preprocess(cloud);
}

// Implement getters
pcl::PointCloud<PointType>::Ptr FeatureExtraction::getCornerPointsSharp() {
    return _cornerPointsSharp;
}

pcl::PointCloud<PointType>::Ptr FeatureExtraction::getCornerPointsLessSharp() {
    return _cornerPointsLessSharp;
}

pcl::PointCloud<PointType>::Ptr FeatureExtraction::getSurfPointsFlat() {
    return _surfPointsFlat;
}

pcl::PointCloud<PointType>::Ptr FeatureExtraction::getSurfPointsLessFlat() {
    return _surfPointsLessFlat;
}

void FeatureExtraction::clearCloud(){
    _cornerPointsSharp->clear();
    _cornerPointsLessSharp->clear();
    _surfPointsFlat->clear();
    _surfPointsLessFlat->clear();
}
