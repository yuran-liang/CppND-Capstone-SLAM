#ifndef POINTCLOUDREADER_H
#define POINTCLOUDREADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

class PointCloudReader {
public:
    PointCloudReader(const std::string &directory);
    
    bool loadNext();
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
    void ClearCloud();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    std::vector<std::string> _fileNames;
    size_t _currentFileIndex;
};

#endif