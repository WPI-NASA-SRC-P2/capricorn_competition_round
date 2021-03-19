#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include <operations/navigation_algorithm.h>


class HeadingMultipleParametersTests :public ::testing::TestWithParam<std::tuple<double, double,
  double, double, double, double, double>> {
};

TEST_P(HeadingMultipleParametersTests, CheckQuat) {
    geometry_msgs::PoseStamped* p = new geometry_msgs::PoseStamped();
    p->pose.orientation.x = std::get<0>(GetParam());
    p->pose.orientation.y = std::get<1>(GetParam());
    p->pose.orientation.z = std::get<2>(GetParam());
    p->pose.orientation.w = std::get<3>(GetParam());
    double expectedRoll = std::get<4>(GetParam());
    double expectedPitch = std::get<5>(GetParam());
    double expectedYaw = std::get<6>(GetParam());
    ASSERT_EQ(expectedRoll, NavigationAlgo::fromQuatToEuler(p)[0]);
    ASSERT_EQ(expectedPitch, NavigationAlgo::fromQuatToEuler(p)[1]);
    ASSERT_EQ(expectedYaw, NavigationAlgo::fromQuatToEuler(p)[2]);
}

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

//Format:(x,y,z,w,roll,pitch,yaw)
INSTANTIATE_TEST_CASE_P(
        DumbTests,
        HeadingMultipleParametersTests,
        ::testing::Values(
                std::make_tuple(0,0,0,1,0,0,0),
                // XYZ
                // 0.283, -0.711, -2.393
                // ZYX
                // 0.296, 0.706, -2.388
                std::make_tuple(-0.369, 0.003, 0.882, -0.294, 0.296, 0.706, -2.388),
                std::make_tuple(0, 0, 0.707, 0.707, 0, 0, M_PI/2))
);

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}