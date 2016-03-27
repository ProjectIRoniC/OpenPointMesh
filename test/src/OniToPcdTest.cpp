
#include <gtest/gtest.h>
#include "oni-to-pcd.h"

#include <boost/filesystem.hpp>


//This is the inherited class of gtest that allows us to automate some work before and after each test
class OniToPcdTest : public ::testing::Test
{

    protected:

        //run this code before each test
        virtual void SetUp()
        {
            this->oniToPcd = new vba::OniToPcd;
        }

        //run this code after each test
        virtual void TearDown()
        {
            delete this->oniToPcd;
        }


        vba::OniToPcd* oniToPcd;

};

//make sure the data members are currently empty
TEST_F( OniToPcdTest , InitialState )
{

}





int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
