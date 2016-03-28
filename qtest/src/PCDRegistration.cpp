

#include "../include/PCDRegistration.h"

namespace vba
{

	PCDRegistration::PCDRegistration( const std::vector< std::string >& files , std::string output_file )
	{
		this->file_list = new std::vector< std::string >( files );
		this->output_filename = output_file;
		this->redirect_output_flag = false;

		this->filter_leaf_size_x = 0.05;
		this->filter_leaf_size_y = 0.05;
		this->filter_leaf_size_z = 0.05;

		this->lock_x_transformations = false;
		this->lock_y_transformations = true;
		this->lock_z_transformations= false;
	}

	PCDRegistration::~PCDRegistration()
	{
		delete this->file_list;
	}

	int PCDRegistration::start()
	{
		std::stringstream output;
		output << "processing " << this->file_list->size() << " frames.\n";
		this->sendOutput( output.str() , false );


		output.str("");
		output << "Loaded " << int( this->file_list->size()) << " datasets\n";
		this->sendOutput( output.str() , false );


		PointCloud::Ptr target( new PointCloud() );
		PointCloud::Ptr result ( new PointCloud() );
		PointCloud::Ptr source( new PointCloud() );
		PointCloud::Ptr final( new PointCloud() );

		Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();

		for (size_t i = 1; i < this->file_list->size() ; ++i)
		{

			pcl::io::loadPCDFile( this->file_list->at( i - 1 ) , *source );
			pcl::io::loadPCDFile( this->file_list->at( i ) , *target );


			PointCloud::Ptr temp (new PointCloud);

			this->pairAlign (source, target, temp, pairTransform, true);

			if( this->lock_x_transformations == true )
				pairTransform( 0 , 3 ) = 0.0f;
			if( this->lock_y_transformations == true )
				pairTransform( 1 , 3 ) = 0.0f;
			if( this->lock_z_transformations == true )
				pairTransform( 2, 3 ) = 0.0f;

			//transform current pair into the global transform
			pcl::transformPointCloud (*temp, *result, GlobalTransform);


			//add cloud to the total model
			*final += *result;
			*final = this->filterCloud( final );

            // output data
            std::stringstream ss;
            ss << "total point count: " << final->size() << "\n";
            this->sendOutput(ss.str(), false);
            // remove contents from stringstream
            ss.str("");

			//update the global transform
			GlobalTransform = GlobalTransform * pairTransform;

			//ss << "temp_cloud" << i << ".pcd";
			//pcl::io::savePCDFile( ss.str() , *final , true );

			ss << "registered cloud ";
			ss << i;
			ss << "/" << this->file_list->size();
			ss << "\n";
			this->sendOutput( ss.str() , false );

		}

		std::stringstream filename;
		filename << this->output_filename << "final_cloud.pcd";
		pcl::io::savePCDFile( filename.str() , *final , true );

//		filename.str("");
//		filename << this->output_filename << "finalPointCloud.ply";
//		pcl::io::savePLYFileASCII( filename.str() , *final ); 

		return 0;
	}

	void PCDRegistration::setFilterLeafSize( float x , float y , float z )
	{
		if( x < 0 || y < 0 || z < 0 )
		{
			this->sendOutput( "Error: cannot specify negative values for filter leaf size.\n" , true );
			return;
		}

		this->filter_leaf_size_x = x;
		this->filter_leaf_size_x = y;
		this->filter_leaf_size_x = z;
	}



	void PCDRegistration::pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample )
	{

		PointCloud::Ptr src (new PointCloud);
		PointCloud::Ptr tgt (new PointCloud);

		*src = this->filterCloud( cloud_src );
		*tgt = this->filterCloud( cloud_tgt );

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

		// *src += *finalCloud;
		// *output = *src;


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

		  //
		  // Transform target back in source frame
		  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

		  *output += *cloud_src;

		  final_transform = targetToSource;

	}


	void PCDRegistration::setOutputBuffer( boost::lockfree::spsc_queue<std::string>* _output_buffer )
	{
		this->output_buffer = _output_buffer;
		this->redirect_output_flag = true;
	}


	void PCDRegistration::sendOutput( std::string output , bool is_error )
	{
		if( this->redirect_output_flag )
		{
			
			if(!this->output_buffer->push(output)) {
                std::cout << "[" << output << "] did not make it too output buffer\n";
			}
		}

		else
		{
			if( is_error == true )
			{
				std::cerr << output;
			}
			else
			{
				std::cout << output;
			}
		}
	}

	PointCloud PCDRegistration::filterCloud( PointCloud::Ptr unfiltered_cloud )
	{
		PointCloud filtered_cloud;

		pcl::VoxelGrid< PointT > voxel_filter;
		voxel_filter.setLeafSize( this->filter_leaf_size_x , this->filter_leaf_size_y , this->filter_leaf_size_z );
		voxel_filter.setInputCloud( unfiltered_cloud );
		voxel_filter.filter( filtered_cloud );

		return filtered_cloud;
	}
}
