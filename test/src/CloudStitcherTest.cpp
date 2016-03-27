
#include <gtest/gtest.h>
#include "CloudStitcher.h"


//This is the inherited class of gtest that allows us to automate some work before and after each test
class CloudStitcherTest : public ::testing::Test
{

    protected:

        //run this code before each test
        virtual void SetUp()
        {
            this->cs = new vba::CloudStitcher;
        }

        //run this code after each test
        virtual void TearDown()
        {
            delete this->cs;
        }


        vba::CloudStitcher* cs;

};

//Testing the setters and getters for the output path
TEST_F( CloudStitcherTest , SettingOutputPath )
{
    cs->setOutputPath( "../res/final_cloud.pcd" );
    EXPECT_STREQ( "../res" , cs->getOutputPath().c_str() );
}

//Make sure the method fails if a path that does not exist is given
TEST_F( CloudStitcherTest , SettingInvalidOutputPath )
{
    int return_code = cs->setOutputPath( "../res/invalid/final_cloud.txt" );
    EXPECT_EQ( -1 , return_code );
}

//Make sure the component only takes a directory name and not a filename
TEST_F( CloudStitcherTest , SettingInvalidFilenameforDirectory )
{
    int return_code = cs->setOutputPath( "../res/empy_cloud.pcd" );
    EXPECT_EQ( -1 , return_code );
}

//Testing if the correct filter leaf size is created from the input value
TEST_F( CloudStitcherTest , SettingFilterResolution )
{
    ASSERT_FLOAT_EQ( 0.1 , cs->getFilterResolution() );

    cs->setFilterResolution( 20 );
    ASSERT_FLOAT_EQ( 0.2 , cs->getFilterResolution() );

    cs->setFilterResolution( 0 );
    ASSERT_FLOAT_EQ( 0.0 , cs->getFilterResolution() );
}

//Testing invalide inputs as the filter resolution
TEST_F( CloudStitcherTest , SettingInvalidFilterResolution )
{
    int return_code = cs->setFilterResolution( 33.2 );
    ASSERT_EQ( -1 , return_code );

    return_code = cs->setFilterResolution( -1 );
    ASSERT_EQ( -1 , return_code );
}

TEST_F( CloudStitcherTest , StitchingPCDFiles )
{
    int return_code = cs->setOutputPath( "../res/final_cloud.pcd" );
    return_code = cs->stitchPCDFiles( "../res/" );

    ASSERT_EQ( 0 , return_code );

}






int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
