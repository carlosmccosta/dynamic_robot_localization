/**\file pointcloud_utils.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/common/pointcloud_utils.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {
namespace pointcloud_utils {


template<typename PointT>
void concatenatePointClouds(std::vector< typename pcl::PointCloud<PointT>::Ptr > pointclouds, typename pcl::PointCloud<PointT>::Ptr pointcloud_out) {
	if (pointcloud_out) {
		for (size_t i = 0; i < pointclouds.size(); ++i) {
			if (pointclouds[i] && !pointclouds[i]->empty()) {
				*pointcloud_out += *(pointclouds[i]);
			}
		}
	}
}


} /* namespace pointcloud_utils */
} /* namespace dynamic_robot_localization */
