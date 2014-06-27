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
	if (features == NULL || features->empty()) {
		PCL_ERROR("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
		return;
	}
	input_features_ = features;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setTargetFeatures(
        const FeatureCloudConstPtr &features) {
	if (features == NULL || features->empty()) {
		PCL_ERROR("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n", getClassName().c_str());
		return;
	}
	target_features_ = features;
	feature_tree_->setInputCloud(target_features_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::selectSamples(
        const PointCloudSource &cloud, int nr_samples, std::vector<int> &sample_indices) {
	if (nr_samples > static_cast<int>(cloud.points.size())) {
		PCL_ERROR("[pcl::%s::selectSamples] ", getClassName().c_str());
		PCL_ERROR("The number of samples (%d) must not be greater than the number of points (%lu)!\n", nr_samples, cloud.points.size());
		return;
	}

	sample_indices.resize(nr_samples);
	int temp_sample;

	// Draw random samples until n samples is reached
	for (int i = 0; i < nr_samples; i++) {
		// Select a random number
		sample_indices[i] = getRandomIndex(static_cast<int>(cloud.points.size()) - i);

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
        const std::vector<int> &sample_indices, std::vector<std::vector<int> >& similar_features, std::vector<int> &corresponding_indices) {
	// Allocate results
	corresponding_indices.resize(sample_indices.size());
	std::vector<float> nn_distances(k_correspondences_);

	// Loop over the sampled features
	for (size_t i = 0; i < sample_indices.size(); ++i) {
		// Current feature index
		const int idx = sample_indices[i];

		// Find the k nearest feature neighbors to the sampled input feature if they are not in the cache already
		if (similar_features[idx].empty()) feature_tree_->nearestKSearch(*input_features_, idx, k_correspondences_, similar_features[idx], nn_distances);

		// Select one at random and add it to corresponding_indices
		if (k_correspondences_ == 1)
			corresponding_indices[i] = similar_features[idx][0];
		else
			corresponding_indices[i] = similar_features[idx][getRandomIndex(k_correspondences_)];
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::computeTransformation(
        PointCloudSource &output, const Eigen::Matrix4f& guess) {
	// Some sanity checks first
	if (!input_features_) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("No source features were given! Call setSourceFeatures before aligning.\n");
		return;
	}
	if (!target_features_) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("No target features were given! Call setTargetFeatures before aligning.\n");
		return;
	}

	if (input_->size() != input_features_->size()) {
		PCL_ERROR("[pcl::%s::computeTransformation] ", getClassName().c_str());
		PCL_ERROR("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n", input_->size(),
		        input_features_->size());
		return;
	}

	if (target_->size() != target_features_->size()) {
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

	// Initialize prerejector (similarity threshold already set to default value in constructor)
	correspondence_rejector_poly_->setInputSource(input_);
	correspondence_rejector_poly_->setInputTarget(target_);
	correspondence_rejector_poly_->setCardinality(nr_samples_);
	setupCorrespondanceRejectors();
	int num_rejections = 0; // For debugging

	// Initialize results
	final_transformation_ = guess;
	inliers_.clear();
	float lowest_error = std::numeric_limits<float>::max();
	converged_ = false;

	// Temporaries
	std::vector<int> inliers;
	float inlier_fraction;
	float error;

	// If guess is not the Identity matrix we check it
	if (!guess.isApprox(Eigen::Matrix4f::Identity(), 0.01f)) {
		PointCloudSource input_transformed;
		input_transformed.resize(input_->size());
		transformPointCloud(*input_, input_transformed, final_transformation_);
		getFitness(input_transformed, inliers, error);
		inlier_fraction = static_cast<float>(inliers.size()) / static_cast<float>(input_->size());
		error /= static_cast<float>(inliers.size());

		if (inlier_fraction >= inlier_fraction_ && error < lowest_error) {
			inliers_ = inliers;
			lowest_error = error;
			converged_ = true;
		}
	}

	// Feature correspondence cache
	std::vector<std::vector<int> > similar_features(input_->size());

	// Start
	for (int i = 0; i < max_iterations_; ++i) {
		// Temporary containers
		std::vector<int> sample_indices, corresponding_indices;

		// Draw nr_samples_ random samples
		selectSamples(*input_, nr_samples_, sample_indices);

		// Find corresponding features in the target cloud
		findSimilarFeatures(sample_indices, similar_features, corresponding_indices);

		pcl::CorrespondencesPtr temp_corrs(new pcl::Correspondences());
		pcl::CorrespondencesPtr filtered_corrs(new pcl::Correspondences());
		for (size_t i = 0; i < sample_indices.size(); ++i) {
			float distance = pcl::euclideanDistance((*input_)[sample_indices[i]], (*target_)[corresponding_indices[i]]);
			temp_corrs->push_back(pcl::Correspondence(sample_indices[i], corresponding_indices[i], distance));
		}

		for (size_t i = 0; i < correspondence_rejectors_.size(); ++i) {
			filtered_corrs = pcl::CorrespondencesPtr(new pcl::Correspondences());
			correspondence_rejectors_[i]->getRemainingCorrespondences(*temp_corrs, *filtered_corrs);
			temp_corrs = filtered_corrs;
		}

		if (filtered_corrs->size() > 2) {
			// Apply prerejection
			/*if (!correspondence_rejector_poly_->thresholdPolygon (sample_indices, corresponding_indices))
		 {
		 ++num_rejections;
		 continue;
		 }*/

			// Estimate the transform from the correspondences, write to transformation_
			//    transformation_estimation_->estimateRigidTransformation(*input_, sample_indices, *target_, corresponding_indices, transformation_);
			transformation_estimation_->estimateRigidTransformation(*input_, *target_, *filtered_corrs, transformation_);


			// Transform the input dataset using the final transformation
			PointCloudSource input_transformed;
			input_transformed.resize(input_->size());
			transformPointCloud(*input_, input_transformed, final_transformation_);

			// Take a backup of previous result
			const Matrix4 final_transformation_prev = final_transformation_;

			// Set final result to current transformation
			final_transformation_ = transformation_;

			// Transform the input and compute the error (uses input_ and final_transformation_)
			getFitness(input_transformed, inliers, error);

			// Restore previous result
			final_transformation_ = final_transformation_prev;

			// If the new fit is better, update results
			inlier_fraction = static_cast<float>(inliers.size()) / static_cast<float>(input_->size());

			// Update result if pose hypothesis is better
			if (inlier_fraction >= inlier_fraction_ && error < lowest_error) {
				inliers_ = inliers;
				lowest_error = error;
				converged_ = true;
				final_transformation_ = transformation_;

				if (update_visualizer_ != 0) {
					std::vector<int> sample_indices_filtered, corresponding_indices_filtered;
					for (size_t i = 0; i < filtered_corrs->size(); ++i) {
						sample_indices_filtered.push_back((*filtered_corrs)[i].index_query);
						corresponding_indices_filtered.push_back((*filtered_corrs)[i].index_match);
					}

					update_visualizer_(input_transformed, sample_indices_filtered, *target_, corresponding_indices_filtered);
				}
			}
		}
	}

	// Apply the final transformation
	if (converged_) transformPointCloud(*input_, output, final_transformation_);

	// Debug output
	PCL_DEBUG("[pcl::%s::computeTransformation] Rejected %i out of %i generated pose hypotheses.\n", getClassName().c_str(), num_rejections, max_iterations_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename FeatureT> void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::getFitness(
		PointCloudSource& input_transformed, std::vector<int>& inliers, float& fitness_score) {
	// Initialize variables
	inliers.clear();
	inliers.reserve(input_->size());
	fitness_score = 0.0f;

	// Use squared distance for comparison with NN search results
	const float max_range = corr_dist_threshold_ * corr_dist_threshold_;

	// For each point in the source dataset
	for (size_t i = 0; i < input_transformed.points.size(); ++i) {
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
	if (inliers.size() > 0)
		fitness_score /= static_cast<float>(inliers.size());
	else
		fitness_score = std::numeric_limits<float>::max();
}

template<typename PointSource, typename PointTarget, typename FeatureT>
void SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::setupCorrespondanceRejectors() {
	pcl::Registration<PointSource, PointTarget>::clearCorrespondenceRejectors();

	typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointSource>::Ptr sac(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointSource>());
	sac->setMaximumIterations(500);
	sac->setInlierThreshold(0.25);
	sac->setInputSource(input_);
	sac->setInputTarget(target_);
	pcl::Registration<PointSource, PointTarget>::addCorrespondenceRejector(sac);
}

} /* namespace dynamic_robot_localization */

#endif

