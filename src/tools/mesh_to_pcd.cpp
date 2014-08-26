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

// project includes
#include <dynamic_robot_localization/common/pointcloud_conversions.h>
#include <dynamic_robot_localization/common/performance_timer.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


void showUsage(char* program_name) {
	pcl::console::print_info("Usage: %s [path/]input.[pcd|obj|ply|stl|vtk] [path/]output.pcd [-binary 0|1]\n", program_name);
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

	pcl::PointCloud<pcl::PointNormal> pointcloud;
	pcl::console::print_highlight("==> Loading %s...\n", argv[1]);
	dynamic_robot_localization::PerformanceTimer performance_timer;
	performance_timer.start();
	if (dynamic_robot_localization::pointcloud_conversions::fromFile(std::string(argv[1]), pointcloud)) {
		pcl::console::print_highlight(" +> Loaded %d points in %s\n", (pointcloud.width * pointcloud.height), performance_timer.getElapsedTimeFormated().c_str());
		pcl::console::print_highlight(" +> Pointcloud fields: %s\n\n", pcl::getFieldsList(pointcloud).c_str());

		pcl::console::print_highlight("==> Saving pointcloud to %s in %s format...\n", argv[2], (binary_output_format ? "binary" : "ascii"));
		performance_timer.restart();
		if (pcl::io::savePCDFile<pcl::PointNormal>(std::string(argv[2]), pointcloud, binary_output_format) == 0) {
			pcl::console::print_highlight(" +> Saved %d points in %s\n\n", (pointcloud.width * pointcloud.height), performance_timer.getElapsedTimeFormated().c_str());
		} else {
			pcl::console::print_error(" !> Failed to save to file %s\n\n", argv[2]);
			showUsage(argv[0]);
			return (-1);
		}
	} else {
		pcl::console::print_error(" !> Failed to load file %s\n\n", argv[1]);
		showUsage(argv[0]);
		return (-1);
	}

	return 0;
}
// ###################################################################################   </main>   #############################################################################
