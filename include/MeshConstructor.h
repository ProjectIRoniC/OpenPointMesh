///////////////////////////////////////////////////////////////////////////////////////
/// @file	MeshConstructor.h
/// @brief	Reads in a .pcd file and applies a mesh to create a new 3D object file
/// @bug	Will try to operate on empty .pcd file
/// @bug    Stray points can significantly alter the accuracy of the final model
///////////////////////////////////////////////////////////////////////////////////////

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

    ///////////////////////////////////////////////////////////////////////////////////////
    /// @brief	Enum containing the filetypes that the output file can be set to
    ///////////////////////////////////////////////////////////////////////////////////////
	enum MESH_FILETYPE
	{
		OBJ,
		VTK,
		PLY
	};

	class MeshConstructor
	{
		public:

            ///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	default constructor
            ///////////////////////////////////////////////////////////////////////////////////////
			MeshConstructor();

			///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	default destructor
            ///////////////////////////////////////////////////////////////////////////////////////
			~MeshConstructor();


            ///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	sets the target input .pcd filename (should be a path if not in current directory)
            ///
            /// @param[in]	filename        		Path to the target .pcd file
            ///
            /// @return	    int			            returns 0 if file exists or -1 otherwise
            ///////////////////////////////////////////////////////////////////////////////////////
			int setInputFilename( std::string filename );


            ///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	sets the output filename path
            ///
            /// @param[in]	filename        		Path to the output file to be created
            /// @param[in]  type                    The filetype to output too. Must be on of the options available in MESH_FILETYPE enum
            /// @return	    int			            returns 0 if successful or -1 otherwise
            ///////////////////////////////////////////////////////////////////////////////////////
			int setOutputFilename( std::string filename , MESH_FILETYPE type );


            ///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	Converts the supplied .pcd file into a mesh file
            ///
            /// @pre        Both the setInputFilename and setOutputFilename function must have been successfully called
            ///
            /// @return	    int			            returns 0 if successful or -1 otherwise
            ///////////////////////////////////////////////////////////////////////////////////////
			int constructMesh();


			///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	get the supplied input .pcd filename
            ///
            /// @return	    string	    The path of the input filename or an empty string if not yet set
            ///////////////////////////////////////////////////////////////////////////////////////
			inline
			std::string getInputFilename() { return input_filename; }


			///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	get the supplied output file path
            ///
            /// @return	    string	    The path of the output path or an empty string if not yet set
            ///////////////////////////////////////////////////////////////////////////////////////
			inline
			std::string getOutputFilename() { return output_filename; }


            ///////////////////////////////////////////////////////////////////////////////////////
            /// @brief	         The standard out and standard error messages can be redirected into a buffer for further processing with this method
            /// @param[in]       boost::lockfree::spsc_queue<std::string>    This is the buffer the messages will be loaded into
            ///////////////////////////////////////////////////////////////////////////////////////
            void setOutputBuffer(boost::lockfree::spsc_queue<std::string>* q);


		private:

			std::string input_filename;
			std::string output_filename;

            boost::lockfree::spsc_queue<std::string>* output_buffer;
            bool redirect_output_flag;

			MESH_FILETYPE output_filetype;



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


	};

} /* namespace vba */

#endif /* MESHCONSTRUCTOR_H_ */
