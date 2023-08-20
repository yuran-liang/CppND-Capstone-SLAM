// Odometry Refinement class

#ifndef ODO_REFINEMENT_H
#define ODO_REFINEMENT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ceres/ceres.h>
#include "helper.h"
#include "LaserOdom.h"
#include <vector>


class OdoRefinement {

public:

  OdoRefinement();

  // Process point cloud data
  void process(std::shared_ptr<LaserOdometry> laserodom);
  void updateodomhighfreq(Eigen::Vector3d t_curr, Eigen::Quaterniond q_curr);
               

  // Functions
  // set initial guess
  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
  void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);

  // Getter
  pcl::PointCloud<PointType>::Ptr GetCloudSurround();
  Eigen::Vector3d GetTwCurr() const;
  Eigen::Quaterniond GetQwCurr() const;
  Eigen::Vector3d GetTwCurrHighFreq() const;
  Eigen::Quaterniond GetQwCurrHighFreq() const;
  std::vector<Eigen::Quaterniond> GetQwPath() const;
  std::vector<Eigen::Vector3d> GetTwPath() const;



private:

  // Frame count
  int _frameCount;   



  // Cube grid parameters
  int _laserCloudCenWidth;
  int _laserCloudCenHeight;
  int _laserCloudCenDepth;

  static inline const int _laserCloudWidth = 21;
  static inline const int _laserCloudHeight = 21;
  static inline const int _laserCloudDepth = 11;
  static inline const int _laserCloudNum = _laserCloudWidth * _laserCloudHeight * _laserCloudDepth;



  // Index of valid cubes
  int _laserCloudValidInd[125];
  int _laserCloudSurroundInd[125];

  // ouput: all visualble cube points
  pcl::PointCloud<PointType>::Ptr _laserCloudSurround;

  // Map point clouds
  pcl::PointCloud<PointType>::Ptr _laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr _laserCloudSurfFromMap;

  // points in every cube
  pcl::PointCloud<PointType>::Ptr _laserCloudCornerArray[_laserCloudNum];
  pcl::PointCloud<PointType>::Ptr _laserCloudSurfArray[_laserCloudNum];

  // KD-trees
  pcl::KdTreeFLANN<PointType>::Ptr _kdtreeCornerFromMap; 
  pcl::KdTreeFLANN<PointType>::Ptr _kdtreeSurfFromMap;

  // Transform 
  double _parameters[7];
  Eigen::Map<Eigen::Quaterniond> _q_w_curr;
  Eigen::Map<Eigen::Vector3d> _t_w_curr;

  // transformation between odom's world and map's world frame
  Eigen::Quaterniond _q_wmap_wodom;
  Eigen::Vector3d _t_wmap_wodom;

  Eigen::Quaterniond _q_wodom_curr;
  Eigen::Vector3d _t_wodom_curr;

  Eigen::Quaterniond _q_wmap_wodom_high_freq;
	Eigen::Vector3d _t_wmap_wodom_high_freq; 

  std::vector<Eigen::Quaterniond> _q_w_path;
  std::vector<Eigen::Vector3d> _t_w_path;


  // Downsampling
  float _lineRes;
  float _planeRes = 0;
  pcl::VoxelGrid<PointType> _downSizeFilterCorner;
  pcl::VoxelGrid<PointType> _downSizeFilterSurf;

  std::vector<int> _pointSearchInd;
  std::vector<float> _pointSearchSqDis;

  PointType _pointOri, _pointSel;

};

#endif