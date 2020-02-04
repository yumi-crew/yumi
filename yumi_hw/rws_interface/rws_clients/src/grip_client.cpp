#include <rws_clients/grip_client.hpp>


namespace rws_clients
{



GripClient::GripClient(std::string name, std::string ns)
: 
Node(name, rclcpp::NodeOptions()), 
goal_done_(false),
namespace_(ns)
{
}

bool
GripClient::init()
{
  using std::placeholders::_1;


  // if global namespace, namespace_ becomes /, giving a invalid topic names. 
  // client should be gripper specific, global gripper clients are therfore not supported.
  if(namespace_.compare("/") == 0)
  {
    return false;
  }

  // Start action client
  action_client_ = rclcpp_action::create_client<Grip>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    namespace_+"/Grip"
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GripClient::send_goal, this)
  );

  return true;
}


bool 
GripClient::is_goal_done() const
{
  return goal_done_;
}


void 
GripClient::perform_grip(int percentage_closed)
{
  percentage_closed_ = percentage_closed;
  send_goal();
}


void 
GripClient::send_goal()
{
  using namespace std::placeholders;

  timer_->cancel();
  goal_done_ = false;


  if (!action_client_) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    goal_done_ = true;
    return;
  }


  auto goal_msg = Grip::Goal();
  goal_msg.grip_percentage_closed = percentage_closed_;


  RCLCPP_INFO(this->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<Grip>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&GripClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =  std::bind(&GripClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&GripClient::result_callback, this, _1);
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
}



void 
GripClient::goal_response_callback(std::shared_future<GoalHandleGrip::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) 
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } 
  else 
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}



void 
GripClient::feedback_callback(GoalHandleGrip::SharedPtr, const std::shared_ptr<const Grip::Feedback> feedback)
{
  RCLCPP_INFO( this->get_logger(),"Feedback on Grip command recieved, Gripper is %d percentage open", (int)feedback->curr_grip_percentage_closed);
}



void 
GripClient::result_callback(const GoalHandleGrip::WrappedResult &result)
{
  

  switch (result.code) 
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received, Gripper is %d percentage open", (int)result.result->res_grip);
  goal_done_ = true;
}



} //end namespace rws_clients