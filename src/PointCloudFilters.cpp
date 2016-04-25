/*
 * PointCloudFilters.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: matt
 */


#include "PointCloudFilters.h"

namespace vba
{

bool STATISTICS_ENABLED = false;

int voxelGridFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 float leaf_size )
{
	if( original_cloud->size() < 10 )
		return -1;

	pcl::VoxelGrid< pcl::PointXYZRGB  > voxel_filter;
	voxel_filter.setLeafSize( leaf_size , leaf_size , leaf_size );
	voxel_filter.setInputCloud( original_cloud  );
	voxel_filter.filter( *filtered_cloud );

	if( STATISTICS_ENABLED == true )
	{
		std::cout << "Original Point Count: " << original_cloud->size() << "\n";
		std::cout << "Filtered Point Count: " << filtered_cloud->size() << "\n";
		int original = original_cloud->size();
		int final = filtered_cloud->size();
		double ratio = double( original / final );
		ratio -= 1.0;
		ratio = ratio * 100;
		std::cout << "Percentage difference: " << ratio << "%\n";

//		pcl::io::savePCDFile( "voxel_filtered_cloud.pcd" , *filtered_cloud );
	}

	return filtered_cloud->size();

}



int radiusOutlierFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 	 pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 	 float radius ,
					 	 unsigned int min_neighbors )
{
	if( original_cloud->size() < 10 )
		return -1;

	if( radius < 0.01 )
		return -1;

	if( min_neighbors < 1 )
		return -1;

	pcl::RadiusOutlierRemoval< pcl::PointXYZRGB > radius_remove;

	radius_remove.setInputCloud( original_cloud );
	radius_remove.setRadiusSearch( radius );
	radius_remove.setMinNeighborsInRadius( min_neighbors  );
	radius_remove.filter( *filtered_cloud );

	if( STATISTICS_ENABLED == true )
	{
		std::cout << "Original Point Count: " << original_cloud->size() << "\n";
		std::cout << "Filtered Point Count: " << filtered_cloud->size() << "\n";
		int original = original_cloud->size();
		int final = filtered_cloud->size();
		double ratio = double( original / final );
		ratio -= 1.0;
		ratio = ratio * 100;
		std::cout << "Percentage difference: " << ratio << "%\n";

//		pcl::io::savePCDFile( "radius_filtered_cloud.pcd" , *filtered_cloud );
	}

	return filtered_cloud->size();
}



int statisticalOutlierFilter( pcl::PointCloud< pcl::PointXYZRGB >::Ptr original_cloud ,
					 	 	  pcl::PointCloud< pcl::PointXYZRGB >::Ptr filtered_cloud ,
					 	 	  float std_dev_mult ,
					 	 	  unsigned int mean_k )
{
	if( original_cloud->size() < 10 )
			return -1;

	pcl::StatisticalOutlierRemoval< pcl::PointXYZRGB > stat_remove;
	stat_remove.setInputCloud( original_cloud );
	stat_remove.setMeanK( mean_k );
	stat_remove.setStddevMulThresh( std_dev_mult );
	stat_remove.filter( *filtered_cloud );

	if( STATISTICS_ENABLED == true )
	{
		std::cout << "Original Point Count: " << original_cloud->size() << "\n";
		std::cout << "Filtered Point Count: " << filtered_cloud->size() << "\n";
		int original = original_cloud->size();
		int final = filtered_cloud->size();
		double ratio = double( original / final );
		ratio -= 1.0;
		ratio = ratio * 100;
		std::cout << "Percentage difference: " << ratio << "%\n";

//		pcl::io::savePCDFile( "statistical_filtered_cloud.pcd" , *filtered_cloud );
	}

	return filtered_cloud->size();
}

}

