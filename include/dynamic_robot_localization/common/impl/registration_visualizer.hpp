/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <dynamic_robot_localization/common/registration_visualizer.h>


namespace dynamic_robot_localization {

template<typename PointSource, typename PointTarget>
RegistrationVisualizer<PointSource, PointTarget>::RegistrationVisualizer() :
	cloud_source_(new pcl::PointCloud<PointTarget>()),
	cloud_target_(new pcl::PointCloud<PointTarget>()),
	cloud_intermediate_(new pcl::PointCloud<PointTarget>()),
	visualizer_updating_mutex_(),
	maximum_displayed_correspondences_(0),
	intermediate_cloud_changed_(false),
	viewport0(0), viewport1(1),
	number_previous_correspondences_(0),
	show_original_source_(false) {}


template<typename PointSource, typename PointTarget>
bool RegistrationVisualizer<PointSource, PointTarget>::setRegistration(
        pcl::Registration<PointSource, PointTarget> &registration) {
	visualizer_updating_mutex_.lock();

	// Update the name of the registration method to be desplayed
	registration_method_name_ = registration.getClassName();

	// Create the local callback function and bind it to the local function resposable for updating
	// the local buffers
	update_visualizer_ = boost::bind(&RegistrationVisualizer<PointSource, PointTarget>::updateIntermediateCloud, this, _1, _2, _3, _4);

	// Register the local callback function to the registration algorithm callback function
	registration.registerVisualizationCallback(this->update_visualizer_);

	// Flag that no visualizer update was done. It indicates to visualizer update function to copy
	// the registration input source and the target point clouds in the next call.

	visualizer_updating_mutex_.unlock();

	return true;
}


template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::startDisplay() {
	// Create and start the rendering thread. This will open the display window.
	viewer_thread_ = boost::thread(&dynamic_robot_localization::RegistrationVisualizer<PointSource, PointTarget>::runDisplay, this);
}


template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::stopDisplay() {
	// Stop the rendering thread. This will kill the display window.
	viewer_thread_.~thread();
}



template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::setSourceCloud(const pcl::PointCloud<PointSource>& cloud_src) {
	visualizer_updating_mutex_.lock();

	*cloud_source_ = cloud_src;
	pcl::visualization::PointCloudColorHandlerCustom<PointSource> cloud_source_handler(cloud_source_, 255, 0, 0);
	if (!viewer_->updatePointCloud<PointSource>(cloud_source_, cloud_source_handler, "cloud source v1")) {
		viewer_->addPointCloud<PointSource>(cloud_source_, cloud_source_handler, "cloud source v1", viewport0);
	}

	visualizer_updating_mutex_.unlock();
}

template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::setTargetCloud(const pcl::PointCloud<PointSource>& cloud_tgt) {
	visualizer_updating_mutex_.lock();

	*cloud_target_ = cloud_tgt;

	pcl::visualization::PointCloudColorHandlerCustom<PointTarget> cloud_target_handler(cloud_target_, 0, 255, 0);
	if (!viewer_->updatePointCloud<PointSource>(cloud_target_, cloud_target_handler, "cloud target v1")) {
		viewer_->addPointCloud<PointTarget>(cloud_target_, cloud_target_handler, "cloud target v1", viewport0);
	}

	if (!viewer_->updatePointCloud<PointSource>(cloud_target_, cloud_target_handler, "cloud target v2")) {
		viewer_->addPointCloud<PointTarget>(cloud_target_, cloud_target_handler, "cloud target v2", viewport1);
	}

	visualizer_updating_mutex_.unlock();
}

template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::setSourceIntermediateCloud(const pcl::PointCloud<PointSource>& cloud_src) {
	visualizer_updating_mutex_.lock();

	*cloud_intermediate_ = cloud_src;
	intermediate_cloud_changed_ = true;

	visualizer_updating_mutex_.unlock();
}

template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::updateViewerIntermediateCloud() {
	pcl::visualization::PointCloudColorHandlerCustom<PointSource> cloud_intermediate_handler_(cloud_intermediate_, 255, 255, 0);
	if (!viewer_->updatePointCloud<PointSource>(cloud_intermediate_, cloud_intermediate_handler_, "cloud intermediate v2")) {
		viewer_->addPointCloud<PointSource>(cloud_intermediate_, cloud_intermediate_handler_, "cloud intermediate v2", viewport1);
	}

	std::string line_root = "line";
	std::string line_name;

	// Remove the old correspondeces
	for (size_t correspondence_id = 0; correspondence_id < number_previous_correspondences_; ++correspondence_id) {
		line_name = getIndexedName(line_root, correspondence_id);
		viewer_->removeShape(line_name, viewport1);
	}

	// Display the new correspondences lines
	size_t correspondences_size = cloud_intermediate_indices_.size();
	std::stringstream stream_;
	stream_ << "Number of correspondences: " << correspondences_size;
	if (!viewer_->updateText(stream_.str(), 10, 70, 0.0, 1.0, 0.0, "correspondences_size")) {
		viewer_->addText(stream_.str(), 10, 70, 0.0, 1.0, 0.0, "correspondences_size", viewport1);
	}

	// Display entire set of correspondece lines if no maximum displayed correspondences is set
	if ((maximum_displayed_correspondences_ > 0) && (maximum_displayed_correspondences_ < correspondences_size))
		correspondences_size = maximum_displayed_correspondences_;

	// Actualize correspondeces_old_size
	number_previous_correspondences_ = correspondences_size;

	// Update new correspondence lines
	for (size_t correspondence_id = 0; correspondence_id < correspondences_size; ++correspondence_id) {
		// Generate random color for current correspondence line
		double random_red = 255 * rand() / (RAND_MAX + 1.0);
		double random_green = 255 * rand() / (RAND_MAX + 1.0);
		double random_blue = 255 * rand() / (RAND_MAX + 1.0);

		if (random_red < 0 || random_red > 255) { random_red = 255; }
		if (random_green < 0 || random_green > 255) { random_green = 255; }
		if (random_blue < 0 || random_blue > 255) { random_blue = 255; }

		// Generate the name for current line
		line_name = getIndexedName(line_root, correspondence_id);

		// Add the new correspondence line.
		viewer_->addLine(
				(*cloud_intermediate_)[cloud_intermediate_indices_[correspondence_id]],
				(*cloud_target_)[cloud_target_indices_[correspondence_id]],
				random_red, random_green, random_blue,
				line_name, viewport1);
	}

	intermediate_cloud_changed_ = false;
}

template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::runDisplay() {
	// Open 3D viewer
	viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_->initCameraParameters();
	viewer_->setCameraPosition(-6, 0, 0, 0, 0, 1);

	if (show_original_source_) {
		// Create the view port for displaying initial source and target point clouds
		viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, viewport0);
		viewer_->setBackgroundColor(0, 0, 0, viewport0);
		viewer_->addText("Initial position of source and target point clouds", 10, 50, "title v1", viewport0);
		viewer_->addText("Green -> target", 10, 30, 0.0, 1.0, 0.0, "legend target v1", viewport0);
		viewer_->addText("Red  -> source", 10, 10, 1.0, 0.0, 0.0, "legend source v1", viewport0);
	}

	// Create the view port for displaying the registration process of source to target point cloud
	viewer_->createViewPort(show_original_source_ ? 0.5 : 0.0, 0.0, 1.0, 1.0, viewport1);
	viewer_->setBackgroundColor(0.0, 0.0, 0.0, viewport1);
	std::string registration_port_title_ = "Registration using " + registration_method_name_;
	viewer_->addText(registration_port_title_, 10, 90, "title v2", viewport1);
	viewer_->addText("Yellow -> intermediate", 10, 50, 1.0, 1.0, 0.0, "legend intermediate v2", viewport1);
	viewer_->addText("Green  -> target", 10, 30, 0.0, 1.0, 0.0, "legend target v2", viewport1);
	viewer_->addText("Red    -> source", 10, 10, 1.0, 0.0, 0.0, "legend source v2", viewport1);

	// Add coordinate system to both ports
	viewer_->addCoordinateSystem();

	// Visualization loop
	while (!viewer_->wasStopped()) {
		// Lock access to visualizer buffers
		visualizer_updating_mutex_.lock();

		if (intermediate_cloud_changed_) {
			updateViewerIntermediateCloud();
		}

		// Render visualizer updated buffers
		viewer_->spinOnce(100);

		// Unlock access to visualizer buffers
		visualizer_updating_mutex_.unlock();

		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::updateIntermediateCloud(
        const pcl::PointCloud<PointSource> &cloud_src, const std::vector<int> &indices_src,
        const pcl::PointCloud<PointTarget> &cloud_tgt, const std::vector<int> &indices_tgt) {
	if (cloud_source_->empty()) {
		setSourceCloud(cloud_src);
	}

	if (cloud_target_->empty()) {
		setTargetCloud(cloud_tgt);
	}

	setSourceIntermediateCloud(cloud_src);

	visualizer_updating_mutex_.lock();
	cloud_intermediate_indices_ = indices_src;
	cloud_target_indices_ = indices_tgt;
	visualizer_updating_mutex_.unlock();
}


template<typename PointSource, typename PointTarget>
void RegistrationVisualizer<PointSource, PointTarget>::setMaximumDisplayedCorrespondences(const int maximum_displayed_correspondences) {
	visualizer_updating_mutex_.lock();
	maximum_displayed_correspondences_ = maximum_displayed_correspondences;
	visualizer_updating_mutex_.unlock();
}


template<typename PointSource, typename PointTarget>
inline std::string RegistrationVisualizer<PointSource, PointTarget>::getIndexedName(std::string &root_name, size_t &id) {
	std::stringstream id_stream_;
	id_stream_ << root_name << id;
	return id_stream_.str();
}

} /* namespace dynamic_robot_localization */
