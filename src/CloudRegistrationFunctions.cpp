

#include "CloudRegistrationFunctions.h"


namespace vba
{


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

    //originally iterates up to 30
    for (int i = 0; i < 10; ++i)
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
        {
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
        }
        prev = reg.getLastIncrementalTransformation ();

    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    final_transform = targetToSource;

}



void pairAlign2(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform )
{
    PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);

	*src = *cloud_src;
	*tgt = *cloud_tgt;


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
	final_transform = registration.getFinalTransformation();


}


void pairAlign3(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform )
{
    PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);

	*src = *cloud_src;
	*tgt = *cloud_tgt;


	// ICP object.
	PointCloud::Ptr finalCloud(new PointCloud);
	pcl::IterativeClosestPointNonLinear<PointT , PointT> registration;
	registration.setInputSource( src );
	registration.setInputTarget( tgt );
	registration.setTransformationEpsilon( 0.001 );

	registration.align(*finalCloud);
	if (registration.hasConverged())
	{
		//std::cout << "ICP converged." << std::endl
		//		  << "The score is " << registration.getFitnessScore() << std::endl;
		//std::cout << "Transformation matrix:" << std::endl;
		//std::cout << registration.getFinalTransformation() << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

	final_transform = registration.getFinalTransformation();


}


void pairAlign4(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform )
{
    PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);

	*src = *cloud_src;
	*tgt = *cloud_tgt;


    pcl::registration::TransformationEstimationSVD< PointT , PointT > te;
    Eigen::Matrix4f reg_transform = Eigen::Matrix4f::Identity();
    te.estimateRigidTransformation( *src , *tgt , reg_transform );


	final_transform = reg_transform;


}





}
