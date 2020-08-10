#include "arena.h"
#include<gtest/gtest.h>

TEST(ArenaService,testInteractionService){
    EXPECT_EQ(1,1);
}

int main(int argc, char const *argv[])
{
    testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"ArenaService");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}
