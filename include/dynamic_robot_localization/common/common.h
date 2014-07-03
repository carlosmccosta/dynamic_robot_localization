#pragma once

/**\file common.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Define all point types that include PointXYZ and Normal data
#define DRL_POINT_TYPES		\
  (pcl::PointNormal)


/*
#define DRL_POINT_TYPES		\
  (pcl::PointNormal)		\
  (pcl::PointXYZRGBNormal)	\
  (pcl::PointXYZINormal)
*/


// Define all point types that represent features
#define DRL_DESCRIPTOR_TYPES \
  (pcl::PFHSignature125) \
  (pcl::FPFHSignature33) \
  (pcl::SHOT352) \
  (pcl::ShapeContext1980) \
  (pcl::ESFSignature640)
//  (typename pcl::Histogram<153>)
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
