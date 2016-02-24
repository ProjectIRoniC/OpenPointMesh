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


int voxelGridFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 float leaf_size );


int radiusOutlierFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 	 pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 	 float radius ,
					 	 unsigned int min_neighbors );


int statisticalOutlierFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 	 	  pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 	 	  float std_dev_mult ,
					 	 	  unsigned int mean_k );

}





#endif /* POINTCLOUDFILTERS_H_ */
