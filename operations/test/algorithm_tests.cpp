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


TEST(DumbTests, SampleHeading)
{
  geometry_msgs::PoseStamped p = new geometry_msgs::PoseStamped();
  p.pose.orientation.x = -0.369;
  p.pose.orientation.y = 0.003;
  p.pose.orientation.z = 0.882;
  p.pose.orientation.w = -0.294;
  //genPose(&p, -0.369, 0.003, 0.882, -0.294);
  ASSERT_EQ(p.pose.orientation.x, -0.369);
  // 163.026
  //ASSERT_EQ(NavigationAlgo::headingFromPose(p), 163.026);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}