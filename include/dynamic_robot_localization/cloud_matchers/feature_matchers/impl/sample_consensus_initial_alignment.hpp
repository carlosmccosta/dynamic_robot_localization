/**\file sample_consensus_initial_alignment.hpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <dynamic_robot_localization/cloud_matchers/feature_matchers/sample_consensus_initial_alignment.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace dynamic_robot_localization {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT, typename FeatureT>
SampleConsensusInitialAlignment<PointT, FeatureT>::SampleConsensusInitialAlignment() :
	matcher_scia_(new SampleConsensusInitialAlignmentRegistration<PointT, FeatureT>()),
	number_of_samples_(3) {
	CloudMatcher<PointT>::setCloudMatcher(matcher_scia_);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <SampleConsensusInitialAlignment-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
template<typename PointT, typename FeatureT>
void SampleConsensusInitialAlignment<PointT, FeatureT>::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle, std::string configuration_namespace) {
	double convergence_time_limit_seconds;
	private_node_handle->param(configuration_namespace + "convergence_time_limit_seconds", convergence_time_limit_seconds, -1.0);

	if (convergence_time_limit_seconds <= 0.0) {
		convergence_time_limit_seconds = std::numeric_limits<double>::max();
	}

	matcher_scia_->setConvergenceTimeLimitSeconds(convergence_time_limit_seconds);

	double min_sample_distance;
	private_node_handle->param(configuration_namespace + "min_sample_distance", min_sample_distance, 1.0);
	matcher_scia_->setMinSampleDistance(min_sample_distance);

	private_node_handle->param(configuration_namespace + "number_of_samples", number_of_samples_, 3);
	matcher_scia_->setNumberOfSamples(number_of_samples_);

	int correspondence_randomness;
	private_node_handle->param(configuration_namespace + "correspondence_randomness", correspondence_randomness, 10);
	matcher_scia_->setCorrespondenceRandomness(correspondence_randomness);

	FeatureMatcher<PointT, FeatureT>::setupConfigurationFromParameterServer(node_handle, private_node_handle, configuration_namespace);
}

template<typename PointT, typename FeatureT>
void SampleConsensusInitialAlignment<PointT, FeatureT>::setMatcherReferenceDescriptors(typename pcl::PointCloud<FeatureT>::Ptr& reference_descriptors) {
	matcher_scia_->setTargetFeatures(reference_descriptors);
}

template<typename PointT, typename FeatureT>
void SampleConsensusInitialAlignment<PointT, FeatureT>::setMatcherAmbientDescriptors(typename pcl::PointCloud<FeatureT>::Ptr& ambient_descriptors) {
	matcher_scia_->setSourceFeatures(ambient_descriptors);
	matcher_scia_->setNumberOfSamples(std::min(number_of_samples_, (int)ambient_descriptors->size()));
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </SampleConsensusInitialAlignment-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} /* namespace dynamic_robot_localization */

