#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cspace.h>

// grid index, width, size, expectedSize, std::vector with neighbor indexes 
class NeighborTests :public ::testing::TestWithParam<std::tuple<int, int, int, int, std::vector<int>>> {
    protected:
    // vars here
    virtual void SetUp() {
        // place setup in here
    }
};
// , std::vector<int>


TEST_P(NeighborTests, GetsAllNeighbors) {
    // this is where we write the actual test
    // this is how we get the param values that are passed in when we instatiate the test
    int gridIndex = std::get<0>(GetParam());
    int gridWidth = std::get<1>(GetParam());
    int gridSize = std::get<2>(GetParam());
    int expectedSize = std::get<3>(GetParam());
    std::vector<int> expectedNeighbors = std::get<4>(GetParam());
    std::vector<int> actualNeighbors = CSpace::get_neighbors_indicies(gridIndex, gridWidth, gridSize);
    // This is where we assert everthing                This seems to be how you output messages
    ASSERT_EQ(expectedSize, actualNeighbors.size()) << "Number of Neighbors is not as expected";

    for (int i = 0; i < actualNeighbors.size(); ++i) {
        ASSERT_EQ(expectedNeighbors[i], actualNeighbors[i]) << "Neighbor at Index " << i << 
                                                                " is not correct expected " << expectedNeighbors[i] << " got " << actualNeighbors[i];
    }

}


INSTANTIATE_TEST_CASE_P(
    // TEST NAME
    NeighborTest,
    // TEST CLASS NAME
    NeighborTests,
    // TESTING VALUES
    ::testing::Values(
        std::make_tuple(12, 5, 25, 8, std::vector<int>{13, 11, 17, 18, 16, 7, 8, 6})

    )
);


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}