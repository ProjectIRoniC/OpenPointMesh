
#include <gtest/gtest.h>
#include "MeshConstructor.h"

#include <boost/filesystem.hpp>


//This is the inherited class of gtest that allows us to automate some work before and after each test
class MeshConstructorTest : public ::testing::Test
{

    protected:

        //run this code before each test
        virtual void SetUp()
        {
            this->meshConstructor = new vba::MeshConstructor;
        }

        //run this code after each test
        virtual void TearDown()
        {
            delete this->meshConstructor;
        }


        vba::MeshConstructor* meshConstructor;

};

//make sure the data members are currently empty
TEST_F( MeshConstructorTest , InitialState )
{
    EXPECT_STREQ( "" , meshConstructor->getInputFilename().c_str() );
    EXPECT_STREQ( "" , meshConstructor->getOutputFilename().c_str() );
}

//Test if a correct filename can be entered and retrieved
TEST_F( MeshConstructorTest , SettingInputFile )
{
    meshConstructor->setInputFilename( "../res/valid_cloud.pcd" );
    EXPECT_STREQ( "../res/valid_cloud.pcd" , meshConstructor->getInputFilename().c_str() );
}

//Test is a bad filename causes an unsucessful return
TEST_F( MeshConstructorTest , SettingInvalidInputFile )
{
    int return_code = meshConstructor->setInputFilename( "myfile.txt" );
    EXPECT_EQ( -1 , return_code );
    EXPECT_STREQ( "" , meshConstructor->getInputFilename().c_str() );
}

//Test if we can detect an empty input cloud
TEST_F( MeshConstructorTest , SettingEmptyInputFile )
{
    int return_code = meshConstructor->setInputFilename( "../res/empty_cloud.pcd" );
    EXPECT_EQ( -1 , return_code );
    EXPECT_STREQ( "" , meshConstructor->getInputFilename().c_str() );
}

//Test if we can successfully set an output filename
TEST_F( MeshConstructorTest , SettingOutputFile )
{
    meshConstructor->setOutputFilename( "final_mesh.ply" , vba::PLY );
    EXPECT_STREQ( "final_mesh.ply" , meshConstructor->getOutputFilename().c_str() );
}

//Testing an invalid output filename given to the component
TEST_F( MeshConstructorTest , SettingInvalidOutputFile )
{
    int return_code = meshConstructor->setOutputFilename( "final_mesh.txt" , vba::PLY );
    EXPECT_EQ( -1 , return_code );
    EXPECT_STREQ( "" , meshConstructor->getOutputFilename().c_str() );
}

//Test if we can start the mesh constructing sequence without first giving a valid input and output filename
TEST_F( MeshConstructorTest , StartMeshConstructionWithoutInput )
{
    int return_code = meshConstructor->constructMesh();
    ASSERT_EQ( -1 , return_code );
}

//Now we will run the mesh construction over a valid input file and then test to make sure the output file matches the name we gave and that it exists
TEST_F( MeshConstructorTest , CheckForFinishedOutputFile )
{
    meshConstructor->setInputFilename( "../res/valid_cloud.pcd" );
    meshConstructor->setOutputFilename( "../res/finished_cloud_mesh.ply" , vba::PLY );
    meshConstructor->constructMesh();

    boost::filesystem::path path( "../res/finished_cloud_mesh.ply" );
    ASSERT_TRUE( boost::filesystem::exists( path ) );
}





int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
