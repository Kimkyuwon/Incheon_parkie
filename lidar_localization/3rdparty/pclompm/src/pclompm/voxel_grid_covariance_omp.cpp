#include "pclompm/voxel_grid_covariance_omp.h"
#include "pclompm/voxel_grid_covariance_omp_impl.hpp"

template class pclompm::VoxelGridCovariance<pcl::PointXYZ>;
template class pclompm::VoxelGridCovariance<pcl::PointXYZI>;
template class pclompm::VoxelGridCovariance<pcl::PointXYZINormal>;
