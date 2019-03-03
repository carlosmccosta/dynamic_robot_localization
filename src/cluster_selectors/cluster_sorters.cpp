/**\file cluster_sorters.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/common.h>
#include <dynamic_robot_localization/cluster_selectors/impl/cluster_sorters.hpp>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifndef DRL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_DRLClusterSorter(T) template class PCL_EXPORTS dynamic_robot_localization::ClusterSorter<T>;
PCL_INSTANTIATE(DRLClusterSorter, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLClusterSizeSorter(T) template class PCL_EXPORTS dynamic_robot_localization::ClusterSizeSorter<T>;
PCL_INSTANTIATE(DRLClusterSizeSorter, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLDistanceToOriginSorter(T) template class PCL_EXPORTS dynamic_robot_localization::DistanceToOriginSorter<T>;
PCL_INSTANTIATE(DRLDistanceToOriginSorter, DRL_POINT_TYPES)

#define PCL_INSTANTIATE_DRLAxisValueSorter(T) template class PCL_EXPORTS dynamic_robot_localization::AxisValueSorter<T>;
PCL_INSTANTIATE(DRLAxisValueSorter, DRL_POINT_TYPES)

#endif
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
