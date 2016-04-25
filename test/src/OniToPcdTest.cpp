
#include <gtest/gtest.h>
#include "oni-to-pcd.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/filesystem.hpp>
#include <string>


//This is the inherited class of gtest that allows us to automate some work before and after each test
class OniToPcdTest : public ::testing::Test
{

    protected:

        //run this code before each test
        virtual void SetUp()
        {
            outputBuffer = new boost::lockfree::spsc_queue<std::string>( 200 );
            this->oniToPcd = new vba::OniToPcd( "../res/res1" , 10 , outputBuffer );
            oniToPcd->setDebugMode( true );
        }

        //run this code after each test
        virtual void TearDown()
        {
            if (this->oniToPcd != NULL){
                delete this->oniToPcd;
            }
            
            if (outputBuffer != NULL){
                delete outputBuffer;
            }
        }


        vba::OniToPcd* oniToPcd;
        boost::lockfree::spsc_queue<std::string>* outputBuffer;

};

/*//Test the default constructor (no arguments)
TEST_F( OniToPcdTest , DefaultConstructor )
{
    EXPECT_STREQ( "" , oniToPcd->OniToPcd() );
}*/

/*//Correct arguments for overloaded constructor
TEST_F( OniToPcdTest , OverloadedConstructor )
{
    
    EXPECT_STREQ( "OniToPcdConstructor worked" , this->outputBuffer.c_str() );
}*/

//Test output buffer creation
TEST_F( OniToPcdTest , BufferCreator )
{
    boost::lockfree::spsc_queue<std::string>* outputBuffer1 = new boost::lockfree::spsc_queue<std::string>( 200 );
    oniToPcd->setOutputBuffer( outputBuffer1 );
    std::string temp = "";
    if(outputBuffer1->empty() == false)
    {
        outputBuffer1->pop(temp);
        EXPECT_STREQ( "setOutputBuffer worked" , temp.c_str() );
    }
    delete outputBuffer1;
}

//Test setFrameSkip;
TEST_F( OniToPcdTest , SetFrameSkip )
{
    unsigned frameSkip = 15;
    oniToPcd->setFrameSkip( frameSkip );
    unsigned return_frameSkip = oniToPcd->getFrameSkip();
    EXPECT_EQ( frameSkip , return_frameSkip );
}

//Test getDebugMode
TEST_F( OniToPcdTest , GetDebugMode )
{
    bool testDebugMode = false;
    oniToPcd->setDebugMode( testDebugMode );
    bool return_testDebugMode = oniToPcd->getDebugMode();
    EXPECT_EQ( testDebugMode , return_testDebugMode );
}

//Test outputOniData incorrect argument
TEST_F( OniToPcdTest , BadOutputOniDataTest )
{
    oniToPcd->outputOniData( "../res/goodDieYoung.oni" );
    std::string temp = "";
    if( outputBuffer->empty() == false ) 
    {
        outputBuffer->pop(temp);
        EXPECT_STREQ( "Couldn't open device" , temp.c_str() );
    }
}

//Test outputOniData correct argument
TEST_F( OniToPcdTest , GoodOutputOniDataTest )
{
    oniToPcd->outputOniData( "../res/StillVideo.oni" );

    boost::filesystem::path path( "../res/res1/StillVideo.oni.csv" );
    ASSERT_TRUE( boost::filesystem::exists( path ) );
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
