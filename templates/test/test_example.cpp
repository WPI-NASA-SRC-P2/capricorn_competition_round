#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include <operations/navigation_algorithm.h>

// Declare a test
TEST(DumbTests, simpleAddition)
{
    ASSERT_EQ(5, 2+3);
    ASSERT_TRUE(true);
}

// Declare another test
TEST(DumbTests, pythagTheorem)
{
    ASSERT_EQ(5, pow(pow(3, 2) + pow(4, 2), 0.5));
}

TEST(DumbTests, otherPackageCall)
{
  ASSERT_EQ(5, NavigationAlgo::getFive());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}