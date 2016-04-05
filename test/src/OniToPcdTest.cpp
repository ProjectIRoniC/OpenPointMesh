
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
    std::cout << "made it here first\n";
    oniToPcd->setOutputBuffer( outputBuffer1 );
    std::string temp = "";
    std::cout << "made it here second\n";
    if(outputBuffer1->empty() == false)
    {
        std::cout << "made the check\n";
        outputBuffer1->pop(temp);
        EXPECT_STREQ( "setOutputBuffer worked" , temp.c_str() );
    }
}

/*//Test setFrameSkip
TEST_F( OniToPcdTest , SetFrameSkip )
{
    unsigned frameSkip = 10;
    EXPECT_STREQ( "" , oniToPcd->setFrameSkip( frameSkip ) );
}*/

//Test outputOniData correct argument
TEST_F( OniToPcdTest , GoodOutputOniDataTest )
{
    oniToPcd->outputOniData( "../res/good1.oni" );
    std::string temp = "";
    std::cout << "made 2nd test\n\n";
    if(outputBuffer->pop(temp) == true)
    {
        std::cout<< "made it here 2nd test\n";
        EXPECT_STREQ( "File open success..." , temp.c_str() );
    }
}

//Test outputOniData incorrect argument
TEST_F( OniToPcdTest , BadOutputOniDataTest )
{
    oniToPcd->outputOniData( "../res/goodDieYoung.oni" );
    oniToPcd->outputOniData( "../res/good1.oni" );
    std::string temp = "";
    if( outputBuffer->pop(temp) == true) 
    {
        EXPECT_STREQ( "Couldn't open device" , temp.c_str() );
    }
}






int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
