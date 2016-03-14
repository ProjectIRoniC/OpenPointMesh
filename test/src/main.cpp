#include <cmath>
#include <gtest/gtest.h>
#include "../../include/CloudStitcher.h"

TEST(SquareRootTest, PositiveNos) {
    ASSERT_EQ(6, std::sqrt(36.0));

    vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;
    ASSERT_EQ( 1 , mCloudStitcher->getNumberofFilesRead() );
    delete mCloudStitcher;
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
