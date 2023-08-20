#include "PointCloudReader.h"
#include <pcl/io/pcd_io.h>
#include <filesystem>


PointCloudReader::PointCloudReader(const std::string& directory) {
    _cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

    // Load all file names in the directory into fileNames
    _currentFileIndex = 0;
    // Use std::filesystem to get filenames of all .pcd files
    for (const auto &entry : std::filesystem::directory_iterator(directory))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd")
        {
            _fileNames.push_back(entry.path().string());
        }
    }
    // Sort filenames to process them in order
    std::sort(_fileNames.begin(), _fileNames.end());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudReader::GetCloud() {return _cloud;}
void PointCloudReader::ClearCloud(){
    _cloud->clear();
}

bool PointCloudReader::loadNext() {
    if (_currentFileIndex >= _fileNames.size()) {
        return false;  // No more files to load
    }

    const auto& filename = _fileNames[_currentFileIndex];
    if (pcl::io::loadPCDFile(filename, *_cloud) == -1) {
    return false;  // Failed to load point cloud
}

    _currentFileIndex++;
    return true;
}
