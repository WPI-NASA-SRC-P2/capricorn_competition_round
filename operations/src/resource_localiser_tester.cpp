#include <operations/ResourceLocaliserAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <utils/common_names.h>

using namespace COMMON_NAMES;

typedef actionlib::SimpleActionClient<operations::ResourceLocaliserAction> ResourceLocaliserClient_;
/**
 *****************************************************
 ***** F I L E   O N L Y   F O R   T E S T I N G *****
 *****************************************************
 (in other words, do not consider this to be good code)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_client");
  ResourceLocaliserClient_ resource_localiser_client_(RESOURCE_LOCALISER_ACTIONLIB, true);
  resource_localiser_client_.waitForServer();
  operations::ResourceLocaliserGoal goal;
  
  resource_localiser_client_.sendGoal(goal);
  resource_localiser_client_.waitForResult(ros::Duration(15.0));
  if (resource_localiser_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", resource_localiser_client_.getState().toString().c_str());
  return 0;
}