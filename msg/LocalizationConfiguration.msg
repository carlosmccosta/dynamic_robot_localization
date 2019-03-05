# Below are the parameter server namespaces that can be provided for reloading each group of the drl processing pipeline (omit leading "/" for using the node local namespace)
# If a given string is empty, the reloading of the associated modules will be skipped
# The drl by default uses a empty configuration_namespace because it loads the configurations from the root of the node local namespace (~)
# As such, use "~" to inform that you want to reload modules from the root of the node local namespace (if a custom namespace is given, the "~" character is not needed since drl uses a private node handle to load its configurations)
string message_management
string general_configurations
string subscribe_topic_names
string service_servers_names
string publish_topic_names
string frame_ids
string reference_pointcloud
string filters
string normal_estimators
string curvature_estimators
string keypoint_detectors
string cloud_matchers_configurations
string initial_pose_estimators_feature_matchers
string initial_pose_estimators_point_matchers
string tracking_matchers
string tracking_recovery_matchers
string transformation_validators_for_tracking
string transformation_validators_for_tracking_recovery
string outlier_detectors
string cloud_analyzers
string registration_covariance_estimators
string tf_publisher
