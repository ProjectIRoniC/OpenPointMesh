/*
 * PointCloudFilters.h
 *
 *  Created on: Jan 28, 2016
 *      Author: matt
 */

#ifndef POINTCLOUDFILTERS_H_
#define POINTCLOUDFILTERS_H_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>


namespace vba
{

///////////////////////////////////////////////////////////////////////////////////////
/// @brief	This filter reduces the density of a cloud by removing points in close vicinity to others
///
/// @param[in]	original_cloud		A pointer to the cloud that needs to be filtered
/// @param[out]	filtered_cloud		After filtering the new cloud will be loaded into this pointer
/// @param[in]	leaf_size			The size of the filtering box that will be applied (typically 0.0-2.0)
///
/// @return     int                 returns 0 if successful or -1 otherwise
///////////////////////////////////////////////////////////////////////////////////////
int voxelGridFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 float leaf_size );



///////////////////////////////////////////////////////////////////////////////////////
/// @brief	This filter removes outlier points that fall outside the radius of the densest parts of the cloud
///
/// @param[in]	original_cloud		A pointer to the cloud that needs to be filtered
/// @param[out]	filtered_cloud		After filtering the new cloud will be loaded into this pointer
/// @param[in]	radius			    The radius applied to each point when searching for other points
/// @param[in]  min_neighbors       The minimum amount of neighbors that must be inside a points radius to be kept in the filtered cloud
///
/// @return     int                 returns 0 if successful or -1 otherwise
///////////////////////////////////////////////////////////////////////////////////////
int radiusOutlierFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 	 pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 	 float radius ,
					 	 unsigned int min_neighbors );



///////////////////////////////////////////////////////////////////////////////////////
/// @brief	This filter removes outlier points that fall outside the radius of the densest parts of the cloud
///
/// @param[in]	original_cloud		A pointer to the cloud that needs to be filtered
/// @param[out]	filtered_cloud		After filtering the new cloud will be loaded into this pointer
/// @param[in]	std_dev_mult	    A standard deviation multiplier used with the mean_k to possible accept points that fall outside the first deviation of mean_k
/// @param[in]  mean_k              Sets the average number of neighbors that a point must contain to be left in the filtered cloud
///
/// @return     int                 returns 0 if successful or -1 otherwise
///////////////////////////////////////////////////////////////////////////////////////
int statisticalOutlierFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 	 	  pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 	 	  float std_dev_mult ,
					 	 	  unsigned int mean_k );

}





#endif /* POINTCLOUDFILTERS_H_ */
