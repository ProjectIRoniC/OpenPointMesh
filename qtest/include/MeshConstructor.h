/*
 * MeshConstructor.h
 *
 *  Created on: Nov 10, 2015
 *      Author: matt
 */

#ifndef MESHCONSTRUCTOR_H_
#define MESHCONSTRUCTOR_H_

#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/poisson.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <sstream>



namespace vba
{

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud< pcl::PointXYZRGB > PointCloud;
	typedef pcl::PointXYZRGBNormal PointNormal;
	typedef pcl::PointCloud< pcl::PointXYZRGBNormal > PointCloudNormals;

	enum MESH_FILETYPE
	{
		OBJ,
		VTK,
		PLY
	};

	class MeshConstructor
	{
		public:

			MeshConstructor();
			~MeshConstructor();

			int setInputFilename( std::string filename );

			int setOutputFilename( std::string filename , MESH_FILETYPE type );

			int constructMesh();
			
			inline
			std::string getInputFilename() { return input_filename; }
			
			inline
			std::string getOutputFilename() { return output_filename; }

            void setOutputBuffer(boost::lockfree::spsc_queue<std::string>* q);

            /*This a function used only in this component. It is called whenever we want to create output messages or
             * error messages. If the user provided an output function we will use that. Otherwise we just print to
             * standard output or standard error.
             *
             * @param: The string containing the message to be outputted.
             * @param: True if the message is an error, false otherwise.
             *
             */
            void sendOutput(std::string output , bool is_error = false );
            void sendOutput(std::stringstream& ss, bool is_error = false);

		private:

			std::string input_filename;
			std::string output_filename;

            boost::lockfree::spsc_queue<std::string>* output_buffer;
            bool redirect_output_flag;

			MESH_FILETYPE output_filetype;


	};

} /* namespace vba */

#endif /* MESHCONSTRUCTOR_H_ */
