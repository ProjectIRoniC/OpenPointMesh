
#include <gtest/gtest.h>
#include <string>
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

}








int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
