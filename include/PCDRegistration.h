

#ifndef PCDREGISTRATION_H_
#define PCDREGISTRATION_H_

//some generic includes
#include <iostream>
#include <vector>
#include <string>

//point cloud data types includes
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

//include to perform standard io
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//includes for point cloud filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

//include for surface normals operations
#include <pcl/features/normal_3d.h>

//includes for point cloud registration (stitching)
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

//include for the actual viewing of point clouds on screen
#include <pcl/visualization/pcl_visualizer.h>

// include for output buffer
#include <boost/lockfree/spsc_queue.hpp>






namespace vba
{


	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointNormal PointNormalT;
	typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

	struct PCD
	{
	  PointCloud::Ptr cloud;
	  std::string filename;

	  PCD() : cloud (new PointCloud) {};
	};

	// Define a new point representation for < x, y, z, curvature >
	class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
	{
	  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
	  MyPointRepresentation ()
	  {
	    // Define the number of dimensions
	    nr_dimensions_ = 4;
	  }

	  // Override the copyToFloatArray method to define our feature vector
	  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
	  {
	    // < x, y, z, curvature >
	    out[0] = p.x;
	    out[1] = p.y;
	    out[2] = p.z;
	    out[3] = p.curvature;
	  }
	};


	class PCDRegistration
	{
		public:


			PCDRegistration( const std::vector< std::string >& files , std::string output_file );
			~PCDRegistration();

			int start();

			/*Public facing function that accepts a function pointer. This function pointer will be passed all
			 * the output from this function. If no function pointer is ever given, we will just default to sending
			 * all the output to standard out and standard error.
			 *
			 * @param: Function pointer following the signature   void functionName( std::string )
			 *
			 */
			void setOutputBuffer( boost::lockfree::spsc_queue<std::string>* output_buffer );

			void setFilterLeafSize( float x , float y , float z );

		private:


			void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample );


			/*This a function used only in this component. It is called whenever we want to create output messages or
			 * error messages. If the user provided an output function we will use that. Otherwise we just print to
			 * standard output or standard error.
			 *
			 * @param: The string containing the message to be outputted.
			 * @param: True if the message is an error, false otherwise.
			 *
			 */
			void sendOutput( std::string output , bool is_error );


			PointCloud filterCloud( PointCloud::Ptr unfiltered_cloud );


			std::vector< std::string >* file_list;
			std::string output_filename;

			boost::lockfree::spsc_queue<std::string>* output_buffer;
			bool redirect_output_flag;

			float filter_leaf_size_x;
			float filter_leaf_size_y;
			float filter_leaf_size_z;

			bool lock_x_transformations;
			bool lock_y_transformations;
			bool lock_z_transformations;

	};

}

#endif
