

#include <string>
#include <vector>
#include <sstream>
#include <boost/lockfree/spsc_queue.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include "PointCloudFilters.h"

//includes for point cloud registration (stitching)
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

//include for surface normals operations
#include <pcl/features/normal_3d.h>

//point cloud data types includes
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>



namespace vba
{

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloud;
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


int openPCDFile( std::string filename , PointCloud::Ptr cloud );





void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform );



void pairAlign2(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform );


void pairAlign3(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform );


void pairAlign4(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform );





}
