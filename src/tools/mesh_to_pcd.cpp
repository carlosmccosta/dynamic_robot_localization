/**\file mesh_to_pcd.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>

// ROS includes

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// external libs includes
#include <Eigen/Core>

// project includes
#include <dynamic_robot_localization/common/pointcloud_conversions.h>
#include <dynamic_robot_localization/common/performance_timer.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


void showUsage(char* program_name) {
	pcl::console::print_info("Usage: %s [path/]input.[pcd|obj|ply|stl|vtk] [path/]output.pcd [-binary 0|1] [-compressed 0|1] [-type PointNormal|PointXYZRGBNormal|auto]\n", program_name);
}


template<typename PointT>
bool savePointCloud(char* output, bool binary_output_format, bool binary_compressed_output_format, PointT& pointcloud) {
	if (binary_compressed_output_format) {
		return pcl::io::savePCDFileBinaryCompressed(std::string(output), pointcloud) == 0;
	} else {
		return pcl::io::savePCDFile(std::string(output), pointcloud, binary_output_format) == 0;
	}
}

template<>
bool savePointCloud(char* output, bool binary_output_format, bool binary_compressed_output_format, pcl::PCLPointCloud2& pointcloud) {
	return pcl::io::savePCDFile(std::string(output), pointcloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary_output_format) == 0;
}


template<typename PointT>
int convertMesh(char* input, char* output, bool binary_output_format, bool binary_compressed_output_format, PointT& pointcloud) {
	pcl::console::print_highlight("==> Loading %s...\n", input);
	dynamic_robot_localization::PerformanceTimer performance_timer;
	performance_timer.start();

	if (dynamic_robot_localization::pointcloud_conversions::fromFile(std::string(input), pointcloud)) {
		pcl::console::print_highlight(" +> Loaded %d points in %s\n", (pointcloud.width * pointcloud.height), performance_timer.getElapsedTimeFormated().c_str());
		pcl::console::print_highlight(" +> Pointcloud fields: %s\n\n", pcl::getFieldsList(pointcloud).c_str());
		std::string save_type = (binary_output_format ? "binary" : "ascii");
		if (binary_compressed_output_format) { save_type += " compressed"; }
		pcl::console::print_highlight("==> Saving pointcloud to %s in %s format...\n", output, save_type.c_str());
		performance_timer.restart();

		if (savePointCloud(output, binary_output_format, binary_compressed_output_format, pointcloud)) {
			pcl::console::print_highlight(" +> Saved %d points in %s taking %s\n\n", (pointcloud.width * pointcloud.height), output, performance_timer.getElapsedTimeFormated().c_str());
		} else {
			pcl::console::print_error(" !> Failed to save to file %s\n\n", output);
		}
	} else {
		pcl::console::print_error(" !> Failed to load file %s\n\n", input);
		return (-1);
	}

	return 0;
}


// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	pcl::console::print_info("###################################################################################\n");
	pcl::console::print_info("############################## Mesh to PCD converter ##############################\n");
	pcl::console::print_info("###################################################################################\n\n");

	if (argc < 3) {
		showUsage(argv[0]);
		return (0);
	}

	bool binary_output_format = true;
	pcl::console::parse_argument(argc, argv, "-binary", binary_output_format);

	bool binary_compressed_output_format = true;
	pcl::console::parse_argument(argc, argv, "-compressed", binary_compressed_output_format);

	std::string type("PointNormal");
	pcl::console::parse_argument(argc, argv, "-type", type);
	/*if (type == "PointNormal") {
		pcl::PointCloud<pcl::PointNormal> pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, binary_compressed_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	} else if (type == "PointXYZRGBNormal") {
		pcl::PointCloud<pcl::PointXYZRGBNormal> pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, binary_compressed_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	} else if (type == "auto") {*/
		pcl::PCLPointCloud2 pointcloud;
		if (convertMesh(argv[1], argv[2], binary_output_format, binary_compressed_output_format, pointcloud) != 0) { showUsage(argv[0]); return (-1); }
	//}

	return 0;
}
// ###################################################################################   </main>   #############################################################################
