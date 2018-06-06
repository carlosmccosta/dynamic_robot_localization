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


template <typename PointT>
void colorizePointCloudWithCurvature(pcl::PointCloud<PointT>& pointcloud) {
	float min_curvature = std::numeric_limits<float>::max();
	float max_curvature = std::numeric_limits<float>::min();

	for (size_t i = 0; i < pointcloud.size(); ++i) {
		if (pointcloud[i].curvature < min_curvature)
			min_curvature = pointcloud[i].curvature;

		if (pointcloud[i].curvature > max_curvature)
			max_curvature = pointcloud[i].curvature;
	}

	float curvature_scale = 360.0f / (max_curvature - min_curvature);

	for (int i = 0; i < pointcloud.size(); ++i) {
		pcl::PointXYZHSV hsv;
		hsv.h = (pointcloud[i].curvature - min_curvature) * curvature_scale;
		hsv.s = 1.0;
		hsv.v = 1.0;
		pcl::PointXYZRGB rgb;
		pcl::PointXYZHSVtoXYZRGB(hsv, rgb);
		pointcloud[i].r = rgb.r;
		pointcloud[i].g = rgb.g;
		pointcloud[i].b = rgb.b;
	}
}

} /* namespace pointcloud_utils */
} /* namespace dynamic_robot_localization */
