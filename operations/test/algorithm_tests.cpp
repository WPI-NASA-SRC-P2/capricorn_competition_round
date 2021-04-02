#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>
#include <operations/navigation_algorithm.h>

class HeadingTests :public ::testing::TestWithParam<std::tuple<double, double,
  double, double, double, double, double, double,
  double, double, double, double>> {
    protected:
    geometry_msgs::PoseStamped* origin;
    geometry_msgs::PoseStamped* target;
      virtual void SetUp() {
        origin = new geometry_msgs::PoseStamped();
        origin->pose.orientation.x = 0.0;
        origin->pose.orientation.y = 0.0;
        origin->pose.orientation.z = 0.0;
        origin->pose.orientation.w = 1.0;
        target = new geometry_msgs::PoseStamped();
        target->pose.orientation.x = 0.0;
        target->pose.orientation.y = 0.0;
        target->pose.orientation.z = 0.0;
        target->pose.orientation.w = 1.0;
      }
};

class PositionTests :public ::testing::TestWithParam<std::tuple<double, double,
  double, double, double, double, double>> {
    protected:
    // Add handling to test poses in different frames of reference
    geometry_msgs::PoseStamped* origin;
    geometry_msgs::PoseStamped* target;
    double distance;
      virtual void SetUp() {
        origin = new geometry_msgs::PoseStamped();
        origin->pose.position.x = 0.0;
        origin->pose.position.y = 0.0;
        origin->pose.position.z = 0.0;
        target = new geometry_msgs::PoseStamped();
        target->pose.position.x = 0.0;
        target->pose.position.y = 0.0;
        target->pose.position.z = 0.0;

        distance = 0.0;
      }
};

TEST_P(HeadingTests, CheckHeadingDelta) {
    target->pose.orientation.x = std::get<0>(GetParam());
    target->pose.orientation.y = std::get<1>(GetParam());
    target->pose.orientation.z = std::get<2>(GetParam());
    target->pose.orientation.w = std::get<3>(GetParam());
    origin->pose.orientation.x = std::get<4>(GetParam());
    origin->pose.orientation.y = std::get<5>(GetParam());
    origin->pose.orientation.z = std::get<6>(GetParam());
    origin->pose.orientation.w = std::get<7>(GetParam());
    double expectedRoll = std::get<8>(GetParam());
    double expectedPitch = std::get<9>(GetParam());
    double expectedYaw = std::get<10>(GetParam());
    double expectedDelta = std::get<11>(GetParam());
    ASSERT_EQ(expectedRoll, NavigationAlgo::fromQuatToEuler(target)[0]);
    ASSERT_EQ(expectedPitch, NavigationAlgo::fromQuatToEuler(target)[1]);
    ASSERT_EQ(expectedYaw, NavigationAlgo::fromQuatToEuler(target)[2]);
}

TEST_P(PositionTests, CheckPositionDelta) {
    geometry_msgs::PoseStamped* target = new geometry_msgs::PoseStamped();
    target->pose.position.x = std::get<0>(GetParam());
    target->pose.position.y = std::get<1>(GetParam());
    target->pose.position.z = std::get<2>(GetParam());
    geometry_msgs::PoseStamped* origin = new geometry_msgs::PoseStamped();
    origin->pose.position.x = std::get<3>(GetParam());
    origin->pose.position.y = std::get<4>(GetParam());
    origin->pose.position.z = std::get<5>(GetParam());
    double distance = std::get<6>(GetParam());
    ASSERT_EQ(distance, NavigationAlgo::changeInPosition(origin, target));
}

class QuaternionTests :public ::testing::TestWithParam<std::tuple<double, double,
  double, double, double, double, double>> {
    protected:
      double epsilon = 0.0001;
};

TEST_P(QuaternionTests, CheckQuat) {
    geometry_msgs::PoseStamped* p = new geometry_msgs::PoseStamped();
    p->pose.orientation.x = std::get<0>(GetParam());
    p->pose.orientation.y = std::get<1>(GetParam());
    p->pose.orientation.z = std::get<2>(GetParam());
    p->pose.orientation.w = std::get<3>(GetParam());
    double expectedRoll = std::get<4>(GetParam());
    double expectedPitch = std::get<5>(GetParam());
    double expectedYaw = std::get<6>(GetParam());
    double aproxRoll = NavigationAlgo::fromQuatToEuler(p)[0];
    double aproxPitch = NavigationAlgo::fromQuatToEuler(p)[1];
    double aproxYaw = NavigationAlgo::fromQuatToEuler(p)[2];
    ASSERT_LE(std::abs(aproxRoll - expectedRoll), epsilon);
    ASSERT_LE(std::abs(aproxPitch - expectedPitch), epsilon);
    ASSERT_LE(std::abs(aproxYaw - expectedYaw), epsilon);
}

//Format:(x,y,z,w,roll,pitch,yaw)
INSTANTIATE_TEST_CASE_P(
        DumbTests,
        QuaternionTests,
        ::testing::Values(
                std::make_tuple(0,0,0,1,0,0,0),
                // XYZ
                // 0.283, -0.711, -2.393
                // ZYX
                // 0.296, 0.706, -2.388
                std::make_tuple(-0.369, 0.003, 0.882, -0.294, 0.296, 0.706, -2.388),
                std::make_tuple(0, 0, 0.707, 0.707, 0, 0, M_PI/2))
);

//Format:(homex,y,z,w,targetx,y,z,w,roll,pitch,yaw,delta)
INSTANTIATE_TEST_CASE_P(
        DumbTests,
        HeadingTests,
        ::testing::Values(
                std::make_tuple(0,0,0,1,0,0,0,1,0,0,0,0),
                std::make_tuple(0,0,0,1,0,0,0,1,0,0,0,0),
                std::make_tuple(0,0,0,1,0,0,0,1,0,0,0,0))
);

//Format:(homex,y,z,targetx,y,z,distance)
INSTANTIATE_TEST_CASE_P(
        DumbTests,
        PositionTests,
        ::testing::Values(
                std::make_tuple(5,4,0,3,2,0,1.4142135623730951),
                std::make_tuple(50,32,0,78,9,0,36.235341863986875),
                std::make_tuple(59.545,-74.1052,0,-15.144,-0.007,0,105.20926748266999))
);

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}