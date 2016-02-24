

#include "CloudRegistrationFunctions.h"


namespace vba
{

int registerPointCloudSet( const std::vector< std::string > file_list , PointCloud::Ptr output_cloud , Eigen::Matrix4f& final_transform , boost::lockfree::spsc_queue<std::string>* buf )
{
	final_transform = Eigen::Matrix4f::Identity();

	std::stringstream output_stream;

	PointCloud::Ptr source( new PointCloud() );
	PointCloud::Ptr target( new PointCloud() );
	PointCloud::Ptr global_cloud( new PointCloud() );
	PointCloud::Ptr final( new PointCloud() );

	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();


	for( unsigned int i = 1 ; i < file_list.size() ; i++ )
	{
		openPCDFile( file_list[ i - 1 ] , source );
		openPCDFile( file_list[ i ] , target );

		//for the algorithms to work correctly, we need all the NaN's to be removed from the clouds
		std::vector< int > indices;
		pcl::removeNaNFromPointCloud( *source , *source , indices );
		pcl::removeNaNFromPointCloud( *target , *target , indices );

		//original is 0.05
		vba::statisticalOutlierFilter( source , source , 1.0 , 5 );
		vba::statisticalOutlierFilter( target , target , 1.0 , 5 );
		vba::voxelGridFilter( source , source , 0.05 );
		vba::voxelGridFilter( target , target , 0.05 );

		//send some output about the progress of stitching so far
		output_stream.str( "" );
		output_stream << "Registering cloud " << i - 1 << " of " << file_list.size() << "\n";
		std::cout << output_stream.str();
		buf->push( output_stream.str() );

		//we input the two clouds to compare and the resulting transformation that lines them up is returned in pairTransform
		vba::pairAlign( source, target, pairTransform );

		//update the global transform with the pairTransform returned from aligning the two clouds
		GlobalTransform = GlobalTransform * pairTransform;

		//transform the target cloud using the accumulated transformations and the one we just got from pairAlign
		std::cout << "Z Translation: " << GlobalTransform( 2 , 2 ) << "\n";
		pcl::transformPointCloud (*target, *target, GlobalTransform);

		//add cloud to the total model. This will append the target's points to the whole cloud we are building up
		*global_cloud += *target;

	}

	final_transform = GlobalTransform;
	*output_cloud = *global_cloud;

	return 0;
}

int openPCDFile( std::string filename , PointCloud::Ptr cloud )
{
	if( pcl::io::loadPCDFile< PointT >( filename , *cloud ) == -1 )
	{
		return -1;
	}

	return 0;
}





void pairAlign( const PointCloud::Ptr cloud_src , const PointCloud::Ptr cloud_tgt , Eigen::Matrix4f &final_transform )
{

	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);

	*src = *cloud_src;
	*tgt = *cloud_tgt;

	/*
	// ICP object.
	PointCloud::Ptr finalCloud(new PointCloud);
	pcl::IterativeClosestPoint<PointT , PointT> registration;
	registration.setInputSource( src );
	registration.setInputTarget( tgt );
	registration.setTransformationEpsilon( 0.01 );

	registration.align(*finalCloud);
	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		//std::cout << "Transformation matrix:" << std::endl;
		//std::cout << registration.getFinalTransformation() << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

	Eigen::Matrix4f reg_transform = Eigen::Matrix4f::Identity();
	reg_transform = registration.getFinalTransformation();
	final_transform = Eigen::Matrix4f::Identity();
	final_transform( 2 , 3 ) = reg_transform( 2 , 3 );


	//
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output, final_transform);

	*output += *cloud_src;

	//*src += *finalCloud;
	//*output = *src;


	*/


	  // Compute surface normals and curvature
	  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	  norm_est.setSearchMethod (tree);
	  norm_est.setKSearch (30);

	  norm_est.setInputCloud (src);
	  norm_est.compute (*points_with_normals_src);
	  pcl::copyPointCloud (*src, *points_with_normals_src);

	  norm_est.setInputCloud (tgt);
	  norm_est.compute (*points_with_normals_tgt);
	  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	  //
	  // Instantiate our custom point representation (defined above) ...
	  MyPointRepresentation point_representation;
	  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	  point_representation.setRescaleValues (alpha);

	  //
	  // Align
	  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	  reg.setTransformationEpsilon (1e-3);
	  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
	  // Note: adjust this based on the size of your datasets
	  reg.setMaxCorrespondenceDistance (0.5);
	  // Set the point representation
	  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	  reg.setInputSource (points_with_normals_src);
	  reg.setInputTarget (points_with_normals_tgt);



	  //
	  // Run the same optimization in a loop and visualize the results
	  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	  reg.setMaximumIterations (2);
	  for (int i = 0; i < 30; ++i)
	  {

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

			//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

			//if the difference between this transformation and the previous one
			//is smaller than the threshold, refine the process by reducing
			//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		  reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();

	  }

		//
	  // Get the transformation from target to source
	  targetToSource = Ti.inverse();

	  final_transform = targetToSource;

}



}
