#include <sg_control/sg_control_sim.h>



namespace sg_control
{


SgControl::SgControl(rclcpp::NodeOptions &options, const std::string &ip)
:
Node("sg_control", options),
ip_(ip)   
{
}


bool
SgControl::init()
{
  //--------------------------------------------------------------------------------------------------------------------
  // Startup. Performs checking of StateMachine's required conditions.
  //--------------------------------------------------------------------------------------------------------------------
  
  //--------------------------------------------------------------------------------------------------------------------
  // Initialize and start action server
  //--------------------------------------------------------------------------------------------------------------------

  using std::placeholders::_1;
  using std::placeholders::_2;
  RCLCPP_INFO(this->get_logger(), "Starting SG_GRIP_CONTROL action server");


  // Using nodegroup namespace to determine which of the grippers this instance is representing
  namespace_ = this->get_namespace();
   
  // Start action server
  action_server_ = rclcpp_action::create_server<Grip>(       
    this->shared_from_this(),
    "Grip",    // Topic names related
    std::bind(&SgControl::handle_goal, this, _1, _2), 
    std::bind(&SgControl::handle_cancel, this, _1),
    std::bind(&SgControl::handle_accepted, this, _1)
  );

  return true;
}



rclcpp_action::GoalResponse 
SgControl::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Grip::Goal> goal)
{
  switch((int)goal->grip_percentage_closed)
  {
    case 0:
      should_grip_in_ = false;
      RCLCPP_INFO(this->get_logger(), "Recieved goal request: Grip Out");
      break;

    case 100:
      should_grip_in_ = true;
      RCLCPP_INFO(this->get_logger(), "Recieved goal request: Grip In");
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Only values 0 and 100 is supported at the moment");
  }
 

  (void)uuid;
  should_execute_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse 
SgControl::handle_cancel(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Not possible to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void 
SgControl::handle_accepted(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  using std::placeholders::_1;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&SgControl::execute, this, _1), goal_handle}.detach();
}


void 
SgControl::execute(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  if(should_grip_in_ && should_execute_)
  {
    perform_grip_in();
    RCLCPP_INFO(this->get_logger(), "Executing Grip In");
  }
  if(!should_grip_in_ && should_execute_)
  {
    perform_grip_out();
    RCLCPP_INFO(this->get_logger(), "Executing Grip Out");
  }
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Grip::Feedback>();
  auto &curr_grip_percentage_closed = feedback->curr_grip_percentage_closed;
  auto result = std::make_shared<Grip::Result>();


  // Since we at the moment have no feedback from the gripper telling if its opened we use an estimate.
  // Estimate: Assuming it takes about 1 second to open gripper. Iterate "10%" every lopp with a loop rate of 6 hz.
  rclcpp::Rate loop_rate(10);
  for (int i = 1; (i < 11) && rclcpp::ok(); ++i) 
  {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) 
    {
      result->res_grip = curr_grip_percentage_closed;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
      should_execute_ = false;
      return;
    }
    // Update percentage
    curr_grip_percentage_closed += 10;
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publishing Feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) 
  {
    result->res_grip = curr_grip_percentage_closed;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    should_execute_ = false;
  }
}


bool
SgControl::is_gripper_open()
{
  // not yet implemented
  return true;
}


//---------- Helper functions ------------------------------------------------------------------------------------------


bool
SgControl::perform_grip_in()
{
  return grip_in();
}


bool
SgControl::perform_grip_out()
{
  return grip_out();
}

bool
SgControl::grip_in()
{
  return true;
}
bool
SgControl::grip_out()
{
  return true;
}

} //end namespace sg_control