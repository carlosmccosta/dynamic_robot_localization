#pragma once

/**\file common.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// Define all point types that include PointXYZ and Normal data
#define DRL_POINT_TYPES					\
  	(pcl::PointXYZRGBNormal)			/*\
	(pcl::PointNormal)					\
	(pcl::PointXYZINormal)				\
	(pcl::PointXYZ)						\ // ->  4 floats
	(pcl::PointXYZI)					\ // ->  8 floats
	(pcl::PointXYZRGB)					\ // ->  8 floats
	(pcl::PointXYZRGBA)					\ // ->  8 floats
	(pcl::PointNormal)					\ // -> 12 floats
	(pcl::PointXYZINormal)				\ // -> 12 floats
	(pcl::PointXYZRGBNormal)	 		  // -> 12 floats*/



// Define all point types that represent features
#define DRL_DESCRIPTOR_TYPES \
	(pcl::PFHSignature125) \
	(pcl::FPFHSignature33) \
	(pcl::SHOT352) \
	(pcl::ShapeContext1980) \
	(pcl::ESFSignature640)
//  (typename pcl::Histogram<153>)


#define DRL_SCALAR_TYPES \
	(float) /*\
	(double)*/

#define DRL_UNPACK_ARGS( ... ) __VA_ARGS__
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
