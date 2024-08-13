#include "pclompm/gicp_omp.h"
#include "pclompm/gicp_omp_impl.hpp"

template class pclompm::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class pclompm::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
template class pclompm::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal>;

