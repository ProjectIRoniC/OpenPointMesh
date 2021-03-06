
#include <gtest/gtest.h>
#include <string>
#include <boost/filesystem.hpp>
#include "filesystemHelper.h"




/// <summary>
/// Creates a name for an output file
/// </summary>
///std::string getOutputFileName(const std::string outputDirectory, const std::string inputFile, const std::string fileExtension);

/// <summary>
/// Creates a directory
/// </summary>
///bool createDirectory(const std::string directoryPath);

/// <summary>
/// Deletes a directory and all its contents if it exists
/// </summary>
///bool deleteDirectory(const std::string directoryPath);



//getOutputFileName Tests
TEST( FilesystemHelperTest , getOutputFileName_HappyPath )
{
    std::string path = vba::filesystemhelper::getOutputFileName( "../res" , "test" , ".pcd" );
    ASSERT_STREQ( "../res/test.pcd" , path.c_str() );
}

TEST( FilesystemHelperTest , getOutputFileName_PassEmptyDir )
{
    std::string path = vba::filesystemhelper::getOutputFileName( "" , "test" , ".pcd" );
    ASSERT_STREQ( "test.pcd" , path.c_str() );
}

TEST( FilesystemHelperTest , getOutputFileName_PassEmptyFilename )
{
    std::string path = vba::filesystemhelper::getOutputFileName( "../res" , "" , ".pcd" );
    ASSERT_STREQ( "" , path.c_str() );
}

TEST( FilesystemHelperTest , getOutputFileName_AddTrailingBackSlashtoDir )
{
    std::string path = vba::filesystemhelper::getOutputFileName( "../res/" , "test" , ".pcd" );
    ASSERT_STREQ( "../res/test.pcd" , path.c_str() );
}

TEST( FilesystemHelperTest , getOutputFileName_EmptyFileExtension )
{
    std::string path = vba::filesystemhelper::getOutputFileName( "../res" , "test" , "" );
    ASSERT_STREQ( "../res/test" , path.c_str() );
}


//createDirectory() Tests

TEST( FilesystemHelperTest , createDirectory_HappyPath )
{
    bool done = vba::filesystemhelper::createDirectory( "../res/test_directory" );

    boost::filesystem::path path( "../res/test_directory" );
    ASSERT_TRUE( boost::filesystem::exists( path ) );
    ASSERT_TRUE( done );
}

TEST( FilesystemHelperTest , createDirectory_TwoLevelDeepDir )
{
    bool done = vba::filesystemhelper::createDirectory( "../res/not_real/test_directory" );

    boost::filesystem::path path( "../res/not_real/test_directory" );
    ASSERT_TRUE( boost::filesystem::exists( path ) );
    ASSERT_TRUE( done );
}


//deleteDirectory() tests

TEST( FilesystemHelperTest , deleteDirectory_HappyPath )
{
    bool done = vba::filesystemhelper::deleteDirectory( "../res/test_directory" );

    boost::filesystem::path path( "../res/test_directory" );
    ASSERT_FALSE( boost::filesystem::exists( path ) );
    ASSERT_TRUE( done );
}


TEST( FilesystemHelperTest , deleteDirectory_NonexistantPath )
{
    bool done = vba::filesystemhelper::deleteDirectory( "../res/not_real/test_directory" );

    ASSERT_FALSE( done );
}







int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
