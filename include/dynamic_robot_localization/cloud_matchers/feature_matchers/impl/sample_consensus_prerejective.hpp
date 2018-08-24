/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_HPP_
#define PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_HPP_

#include <dynamic_robot_localization/cloud_matchers/feature_matchers/sample_consensus_prerejective.h>

namespace dynamic_robot_localization {

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setSourceFeatures(
        const FeatureCloudConstPtr &features) {
	if (!features || features->empty()) {
		PCL_ERROR("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
		return;
	}
	input_features_ = features;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setTargetFeatures(
        const FeatureCloudConstPtr &features) {
	if (!features || features->empty()) {
		PCL_ERROR("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
		return;
	}
	target_features_ = features;
	feature_tree_->setInputCloud(target_features_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::selectSamples(
        const PointCloudSource &cloud, int nr_samples, std::vector<int> &sample_indices) {
	if (nr_samples > static_cast<int>(cloud.size())) {
		PCL_ERROR("[pcl::%s::selectSamples] ", getClassName().c_str());
		PCL_ERROR("The number of samples (%d) must not be greater than the number of points (%lu)!\n", nr_samples, cloud.size());
		return;
	}

	sample_indices.resize(nr_samples);
	int temp_sample;

	// Draw random samples until n samples is reached
	for (int i = 0; i < nr_samples; i++) {
		// Select a random number
		sample_indices[i] = getRandomIndex(static_cast<int>(cloud.size()) - i);

		// Run trough list of numbers, starting at the lowest, to avoid duplicates
		for (int j = 0; j < i; j++) {
			// Move value up if it is higher than previous selections to ensure true randomness
			if (sample_indices[i] >= sample_indices[j]) {
				sample_indices[i]++;
			} else {
				// The new number is lower, place it at the correct point and break for a sorted list
				temp_sample = sample_indices[i];
				for (int k = i; k > j; k--)
					sample_indices[k] = sample_indices[k - 1];

				sample_indices[j] = temp_sample;
				break;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::findSimilarFeatures(
        const std::vector<int> &sample_indices, std::vector<int> &corresponding_indices) {
	// Allocate results
	corresponding_indices.resize(sample_indices.size(), 0);
	int k = std::min(k_correspondences_, (int)target_features_->size());
	std::vector<int> similar_features(k);
	std::vector<float> nn_distances(k);

	// Loop over the sampled features
	for (size_t i = 0; i < sample_indices.size(); ++i) {
		// Current feature index
		const int idx = sample_indices[i];

		int number_k_found = feature_tree_->nearestKSearch(*input_features_, idx, k, similar_features, nn_distances);

		if (number_k_found > 0) {
			if (k == 1)
				corresponding_indices[i] = similar_features[0];
			else
				corresponding_indices[i] = similar_features[getRandomIndex(number_k_found)];
		} else {
			corresponding_indices[i] = 0;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::computeTransformation(
        PointCloudSource &output, const Eigen::Matrix4f& guess) {
	// Some sanity checks first
	if (!input_features_ || !input_ || input_features_->empty()) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("No source features were given! Call setSourceFeatures before aligning.\n");
		return;
	}
	if (!target_features_ || !target_ || target_features_->empty()) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("No target features were given! Call setTargetFeatures before aligning.\n");
		return;
	}

	if (input_->size() != input_features_->size() || input_->empty()) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n", input_->size(),
		        input_features_->size());
		return;
	}

	if (target_->size() != target_features_->size() || target_->empty()) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n", target_->size(),
		        target_features_->size());
		return;
	}

	if (inlier_fraction_ < 0.0f || inlier_fraction_ > 1.0f) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal inlier fraction %f, must be in [0,1]!\n", inlier_fraction_);
		return;
	}

	const float similarity_threshold = correspondence_rejector_poly_->getSimilarityThreshold();
	if (similarity_threshold < 0.0f || similarity_threshold >= 1.0f) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal prerejection similarity threshold %f, must be in [0,1[!\n", similarity_threshold);
		return;
	}

	if (k_correspondences_ <= 0) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal correspondence randomness %d, must be > 0!\n", k_correspondences_);
		return;
	}

	if (nr_samples_ < 3) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("Illegal nr_samples %d, must be >= 3!\n", nr_samples_);
		return;
	}

	convergence_timer_.reset();

	// Initialize prerejector (similarity threshold already set to default value in constructor)
	/*correspondence_rejector_poly_->setInputSource(input_);
	correspondence_rejector_poly_->setInputTarget(target_);
	correspondence_rejector_poly_->setCardinality(nr_samples_);*/
	pcl::Registration<PointSource, PointTarget>::clearCorrespondenceRejectors();
	setupCorrespondanceRejectors(correspondence_rejectors_);
	int num_rejections = 0; // For debugging

	// Initialize results
	final_transformation_ = guess;
	inliers_.clear();
	double lowest_error = std::numeric_limits<double>::max();
	converged_ = false;


	// If guess is not the Identity matrix we check it
	if (!guess.isApprox(Eigen::Matrix4f::Identity(), 0.01f)) {
		std::vector<int> inliers;
		float inlier_fraction = 0.0;
		double error = std::numeric_limits<double>::max();

		PointCloudSource input_transformed;
		input_transformed.resize(input_->size());
		transformPointCloudWithNormals(*input_, input_transformed, final_transformation_);
		getFitness(input_transformed, inliers, error);
		if (!inliers.empty()){
			error /= static_cast<double>(inliers.size());
			if (input_->empty()) {
				inlier_fraction = 0.0;
			} else {
				inlier_fraction = static_cast<double>(inliers.size()) / static_cast<double>(input_->size());
			}
		}

		if (inlier_fraction >= inlier_fraction_ && error < lowest_error) {
			inliers_ = inliers;
			lowest_error = error;
			converged_ = true;
		}
	}


#ifdef USE_GROUPING
	//
	//  Find Model-Scene Correspondences with KdTree
	//
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < input_features_->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl::isFinite(input_features_->at(i))) { continue; }
		if (!pcl::isFinite(input_->at(i)) || !pcl::isFinite(target_->at(i))) { continue; }

		int found_neighs = feature_tree_->nearestKSearch (input_features_->at(i), 1, neigh_indices, neigh_sqr_dists);
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) { //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;


	//
	// filter outliers
	//
	pcl::CorrespondencesPtr filtered_corrs(new pcl::Correspondences());
	for (size_t i = 0; i < correspondence_rejectors_.size(); ++i) {
		filtered_corrs = pcl::CorrespondencesPtr(new pcl::Correspondences());
		correspondence_rejectors_[i]->getRemainingCorrespondences(*model_scene_corrs, *filtered_corrs);
		if (filtered_corrs->size() < 3) break;
		model_scene_corrs = filtered_corrs;
	}
	model_scene_corrs = filtered_corrs;

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	#ifdef USE_HOUGH
	if (use_hough_) {
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());

		pcl::BOARDLocalReferenceFrameEstimation<pcl::PointNormal, pcl::PointNormal, pcl::ReferenceFrame> rf_est;
		rf_est.setFindHoles (true);
		rf_est.setRadiusSearch (0.25);

		rf_est.setInputCloud (target_);
		rf_est.setInputNormals (target_);
//		rf_est.setSearchSurface (model);
		rf_est.compute (*model_rf);

		rf_est.setInputCloud (input_);
		rf_est.setInputNormals (input_);
//		rf_est.setSearchSurface (scene);
		rf_est.compute (*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<pcl::PointNormal, pcl::PointNormal, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
		clusterer.setHoughBinSize(0.01);
		clusterer.setHoughThreshold(5);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);

		clusterer.setInputCloud(target_);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(input_);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize (rototranslations, clustered_corrs);
	#else
		pcl::GeometricConsistencyGrouping<PointSource, PointTarget> gc_clusterer;
		gc_clusterer.setGCSize(0.01);
		gc_clusterer.setGCThreshold(5);

		gc_clusterer.setInputCloud(target_);
		gc_clusterer.setSceneCloud(input_);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
		gc_clusterer.recognize(rototranslations, clustered_corrs);
	#endif


	if (update_visualizer_ != 0) {
		std::vector<int> sample_indices_filtered, corresponding_indices_filtered;
		for (size_t i = 0; i < model_scene_corrs->size(); ++i) {
			sample_indices_filtered.push_back((*model_scene_corrs)[i].index_query);
			corresponding_indices_filtered.push_back((*model_scene_corrs)[i].index_match);
		}

		update_visualizer_(*input_, sample_indices_filtered, *target_, corresponding_indices_filtered);
	}

	for (int i = 0; i < rototranslations.size(); ++i) {
		if (clustered_corrs[i].size() > 2) {
			// Estimate the transform from the correspondences, write to transformation_
			//    transformation_estimation_->estimateRigidTransformation(*input_, sample_indices, *target_, corresponding_indices, transformation_);
//			transformation_estimation_->estimateRigidTransformation(*input_, *target_, *filtered_corrs, transformation_);

			// Transform the input dataset using the final transformation
			PointCloudSource input_transformed;
			input_transformed.resize(input_->size());
			transformPointCloudWithNormals(*input_, input_transformed, rototranslations[i]);

			// Transform the input and compute the error (uses input_ and final_transformation_)
			getFitness(input_transformed, inliers, error);

			// If the new fit is better, update results
			inlier_fraction = static_cast<float>(inliers.size()) / static_cast<float>(input_->size());

			if (update_visualizer_ != 0) {
				std::vector<int> sample_indices_filtered, corresponding_indices_filtered;
				for (size_t j = 0; j < clustered_corrs[i].size(); ++j) {
					sample_indices_filtered.push_back(clustered_corrs[i][j].index_query);
					corresponding_indices_filtered.push_back(clustered_corrs[i][j].index_match);
				}

				update_visualizer_(input_transformed, sample_indices_filtered, *target_, corresponding_indices_filtered);
			}

			// Update result if pose hypothesis is better
			if (inlier_fraction >= inlier_fraction_ && error < lowest_error) {
				inliers_ = inliers;
				lowest_error = error;
				converged_ = true;
				final_transformation_ = transformation_;
			}
		}
	}

#else //-----------------------------------------------------------------------------------------------------------------------------------

	double highest_inlier_fraction = 0.0;
	accepted_transformations_->clear();

	#pragma omp parallel for
	for (int i = 0; i < max_iterations_; ++i) {
		if (convergence_timer_.getTimeSeconds() > convergence_time_limit_seconds_) {
			continue;
		}

		//		if (highest_inlier_fraction < 0.99) {
			std::vector<int> sample_indices, corresponding_indices;

			// Draw nr_samples_ random samples
			selectSamples(*input_, nr_samples_, sample_indices);

			// Find corresponding features in the target cloud
			findSimilarFeatures(sample_indices, corresponding_indices);

			// Apply prerejection
			/*if (!correspondence_rejector_poly_->thresholdPolygon (sample_indices, corresponding_indices)) {
			++num_rejections;
			continue;
			}*/

			pcl::CorrespondencesPtr temp_corrs(new pcl::Correspondences());
			pcl::CorrespondencesPtr filtered_corrs(new pcl::Correspondences());
			for (size_t i = 0; i < sample_indices.size(); ++i) {
				float distance = pcl::euclideanDistance((*input_)[sample_indices[i]], (*target_)[corresponding_indices[i]]);
				temp_corrs->push_back(pcl::Correspondence(sample_indices[i], corresponding_indices[i], distance));
			}

			if (temp_corrs->empty()) continue;


			// correspondence grouping
			// TODO: aa

			std::vector< typename pcl::registration::CorrespondenceRejector::Ptr > correspondence_rejectors;
			setupCorrespondanceRejectors(correspondence_rejectors);

			for (size_t i = 0; i < correspondence_rejectors.size(); ++i) {
				filtered_corrs = pcl::CorrespondencesPtr(new pcl::Correspondences());

				//			#pragma omp critical
				correspondence_rejectors[i]->getRemainingCorrespondences(*temp_corrs, *filtered_corrs);

				if (filtered_corrs->size() < 3) break;
				temp_corrs = filtered_corrs;
			}

			if (filtered_corrs->size() > 2) {
				Matrix4 transformation;
				pcl::registration::TransformationEstimationSVD<PointSource, PointTarget> transformation_estimation;

				// Estimate the transform from the correspondences, write to transformation_
				//    transformation_estimation_->estimateRigidTransformation(*input_, sample_indices, *target_, corresponding_indices, transformation_);
				//			#pragma omp critical
				transformation_estimation.estimateRigidTransformation(*input_, *target_, *filtered_corrs, transformation);

				// Transform the input dataset using the final transformation
				PointCloudSource input_transformed;
				pcl::transformPointCloudWithNormals(*input_, input_transformed, transformation);

				std::vector<int> inliers;
				double error;

				// Transform the input and compute the error (uses input_ and final_transformation_)
				getFitness(input_transformed, inliers, error);

				if (inliers.size() > 2) {
					double current_inlier_fraction = 0.0;
					if (!input_->empty()) {
						current_inlier_fraction = static_cast<double>(inliers.size()) / static_cast<double>(input_->size());
					}

					if (update_visualizer_ != 0) {
						std::vector<int> sample_indices_filtered, corresponding_indices_filtered;
						for (size_t i = 0; i < filtered_corrs->size(); ++i) {
							sample_indices_filtered.push_back((*filtered_corrs)[i].index_query);
							corresponding_indices_filtered.push_back((*filtered_corrs)[i].index_match);
						}
						#pragma omp critical
						update_visualizer_(input_transformed, sample_indices_filtered, *target_, corresponding_indices_filtered);
					}

					// Update result if pose hypothesis is better
					#pragma omp critical
					if (current_inlier_fraction >= inlier_fraction_ && error < inlier_rmse_) {
						accepted_transformations_->push_back(transformation);
						if (error < lowest_error) {
							highest_inlier_fraction = current_inlier_fraction;
							inliers_ = inliers;
							lowest_error = error;
							converged_ = true;
							final_transformation_ = transformation;
							transformation_ = transformation;
						}
					}
				}
			}
//		}
	}
#endif //--------------------------------------------------------------------------------------------------------------------------------



	// Apply the final transformation
	output.clear();
	if (converged_) pcl::transformPointCloudWithNormals(*input_, output, final_transformation_);

	// Debug output
	PCL_DEBUG("[pcl::%s::computeTransformation] Accepted %i out of %i generated pose hypotheses.\n", getClassName().c_str(), accepted_transformations_->size(), max_iterations_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::getFitness(
		PointCloudSource& input_transformed, std::vector<int>& inliers, double& fitness_score) {
	// Initialize variables
	inliers.clear();
	inliers.reserve(input_->size());
	fitness_score = 0.0f;

	// Use squared distance for comparison with NN search results
	const float max_range = corr_dist_threshold_ * corr_dist_threshold_;

	// For each point in the source dataset
	for (size_t i = 0; i < input_transformed.size(); ++i) {
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range) {
			// Update inliers
			inliers.push_back(static_cast<int>(i));

			// Update fitness score
			fitness_score += nn_dists[0];
		}
	}

	// Calculate MSE
	if (inliers.size() > 0) {
		fitness_score /= static_cast<double>(inliers.size());
		fitness_score = std::sqrt(fitness_score);
	} else {
		fitness_score = std::numeric_limits<double>::max();
	}
}

template<typename PointSource, typename PointTarget, typename FeatureT>
void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setupCorrespondanceRejectors(std::vector< typename pcl::registration::CorrespondenceRejector::Ptr >& correspondence_rejectors) {
	typename pcl::registration::CorrespondenceRejectorOneToOne::Ptr corr_rej_o2o(new pcl::registration::CorrespondenceRejectorOneToOne());
	correspondence_rejectors.push_back(corr_rej_o2o);


	/*typename pcl::registration::CorrespondenceRejectorMedianDistance::Ptr corr_rej_median (new pcl::registration::CorrespondenceRejectorMedianDistance);
	corr_rej_median->setInputSource<PointSource>(input_);
	corr_rej_median->setInputTarget<PointTarget>(target_);
	corr_rej_median->setMedianFactor(4.0);
	pcl::Registration<PointSource, PointTarget>::addCorrespondenceRejector(corr_rej_median);*/


	/*typename pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr corr_rej_var (new typename pcl::registration::CorrespondenceRejectorVarTrimmed());
	corr_rej_var->setInputSource<PointSource>(input_);
	corr_rej_var->setInputTarget<PointTarget>(target_);
	pcl::Registration<PointSource, PointTarget>::addCorrespondenceRejector(corr_rej_var);*/


	/*typename pcl::registration::CorrespondenceRejectorTrimmed::Ptr corr_rej_tri(new typename pcl::registration::CorrespondenceRejectorTrimmed());
	pcl::Registration<PointSource, PointTarget>::addCorrespondenceRejector(corr_rej_tri);*/


	typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointSource>::Ptr corr_rej_sac(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointSource>());
	corr_rej_sac->setInputSource(input_);
	corr_rej_sac->setInputTarget(target_);
	corr_rej_sac->setInlierThreshold(0.25);
	corr_rej_sac->setMaximumIterations(500);
	correspondence_rejectors.push_back(corr_rej_sac);
}

} /* namespace dynamic_robot_localization */

#endif

