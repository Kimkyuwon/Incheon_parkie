#include "pclompm/ndt_omp.h"
#include "pclompm/ndt_omp_impl.hpp"

template class pclompm::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclompm::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclompm::NormalDistributionsTransform<pcl::PointXYZINormal, pcl::PointXYZINormal>;
