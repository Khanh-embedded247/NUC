#include "robot_control_nav2/moving_pose_srv_server.hpp"

MovingPoseSrvServer::MovingPoseSrvServer()
    : Node("moving_pose_srv_server_node")
{

  RCLCPP_INFO_STREAM(this->get_logger(), "moving_pose_srv_server_node");
  // Khởi tạo tf_buffer_ để lưu trữ transform và tf_listener_ để lắng nghe các transform
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publisher
  _response_pub = this->create_publisher<std_msgs::msg::Int32>("response", 10);
  cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  next_point_res_pub = this->create_publisher<std_msgs::msg::Int32>("next_point_res", 10);

  // Subscriber
  pose_robot_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose_robot", 10,
                                                                              std::bind(&MovingPoseSrvServer::curent_pose_robot_cb, this, std::placeholders::_1));

  next_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("next_pose_req", 10,
                                                                             std::bind(&MovingPoseSrvServer::next_pose_cb, this, std::placeholders::_1));

  // // Service server
  // _server = this->create_service<robot_msgs::srv::OperatorTwoInts>("operator_two_ints",
  //                                                                  std::bind(&MovingPoseSrvServer::operator_two_ints_callback, this,
  //                                                                            std::placeholders::_1, std::placeholders::_2));

  // Service server để nhận yêu cầu di chuyển tới một vị trí cụ thể
  navigate_pose_server = this->create_service<robot_msgs::srv::NavigatePose>("navigate_to_pose",
                                                                             std::bind(&MovingPoseSrvServer::navigate_pose_server_cb, this,
                                                                                       std::placeholders::_1, std::placeholders::_2));

  // Tạo các client để gửi yêu cầu để xóa toàn bộ costmap cục bộ và toàn cầu
  clear_local_cost_map_cl = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
  clear_global_cost_map_cl = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");

  // Tạo action client để gửi yêu cầu di chuyển tới một vị trí cụ thể
  client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

  // this->timer_ = this->create_wall_timer(
  //       std::chrono::milliseconds(2000),
  //       std::bind(&MovingPoseSrvServer::timer_callback, this));

  rclcpp::Rate loop_rate_(1);
  //Lặp lại việc kiểm tra xem có thể nhận được transform từ tf không. Nếu không, tiếp tục lặp.
  while (!get_pose_robot())
  {
    RCLCPP_INFO(this->get_logger(), "Could not transform ");

    loop_rate_.sleep();
  }

  clear_all_costmap();
  loop_rate_.sleep();
  next_point_res = 0; // MODE IDEL
  next_point_res_data.data = next_point_res;
  // publish mode robot
  next_point_res_pub->publish(next_point_res_data);

  // =====================
  // send_goal_pose();
}
void MovingPoseSrvServer::timer_callback()
{
}

void MovingPoseSrvServer::send_goal_pose()
{
  // Gửi một mục tiêu di chuyển tới robot
  // Đảm bảo rằng action server có sẵn để nhận yêu cầu
  RCLCPP_ERROR(this->get_logger(), "Send Goal Pose");
  if (!this->client_ptr_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  // geometry_msgs::msg::PoseStamped goal_pose;
  //  x : 14.503415  y :-21.736702  z :-0.746313  w:0.665595
  //  x : 14.503415  y :-21.736702  z :-0.067894  w:0.997693
  // x : 14.503415  y :-21.736702  z :0.175506  w:0.984478

  // goal_pose.header.frame_id = "map";
  // goal_pose.header.stamp = this->get_clock()->now();
  // goal_pose.pose.position.x = 14.503415;
  // goal_pose.pose.position.y = -21.736702 ;
  // goal_pose.pose.position.z = 0.0;

  // goal_pose.pose.orientation.x = 0.0;
  // goal_pose.pose.orientation.y = 0.0;
  // goal_pose.pose.orientation.z = 0.175506;
  // goal_pose.pose.orientation.w = 0.984478 ;
  // Tạo một goal pose mới
  target_pose_robot = goal_pose;
  goal_data.pose = target_pose_robot;
  // Lấy vị trí hiện tại của robot
  get_pose_robot();

  // Tính toán góc xoay để định hướng robot di chuyển
  curent_theta = quat_to_yaw(curent_pose_robot);
  target_theta = std::atan2((target_pose_robot.pose.position.y - curent_pose_robot.pose.position.y),
                            (target_pose_robot.pose.position.x - curent_pose_robot.pose.position.x));
   // Lấy khoảng cách tới mục tiêu
      double distance_to_target = std::sqrt(std::pow((target_pose_robot.pose.position.x - curent_pose_robot.pose.position.x), 2) +
                                            std::pow((target_pose_robot.pose.position.y - curent_pose_robot.pose.position.y), 2));
      RCLCPP_INFO(this->get_logger(), "Dístance : %f" ,istance_to_target);
  // Tạo dữ liệu cho tốc độ di chuyển
  geometry_msgs::msg::Twist cmd_vel_data;

  clear_all_costmap();
  rclcpp::Rate loop_rate_(1);
  loop_rate_.sleep();

  // Nếu góc giữa vị trí hiện tại và mục tiêu di chuyển >11,46 theo trucj x dương và <11,46 độ theo trục x âm,
  // robot sẽ xoay để hướng về mục tiêu
  if ((std::abs(target_theta - curent_theta) > error_theta) &&
      (std::abs(target_theta - curent_theta) < (2 * pi_ - error_theta)))
  {
    rclcpp::Rate loop_rate(10);

    while ((std::abs(target_theta - curent_theta) > error_theta) &&
           (std::abs(target_theta - curent_theta) < (2 * pi_ - error_theta)))
    {
      // Lấy lại vị trí hiện tại của robot

      get_pose_robot();

      curent_theta = quat_to_yaw(curent_pose_robot);

      target_theta = std::atan2((target_pose_robot.pose.position.y - curent_pose_robot.pose.position.y),
                                (target_pose_robot.pose.position.x - curent_pose_robot.pose.position.x));
     


      //  Quay trái nếu góc quay cần  là dương
      if (((target_theta - curent_theta) > error_theta) && ((target_theta - curent_theta) < (pi_)) ||
          ((target_theta - curent_theta) > (-2.0 * pi_)) && ((target_theta - curent_theta) < (-pi_)))
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Turn Left :" << (target_theta - curent_theta));
        cmd_vel_data.angular.z = 0.3;//~17,2 độ
      }
      // Quay phải nếu góc quay cần  là âm
      if (((target_theta - curent_theta) > (pi_)) && ((target_theta - curent_theta) < (2 * pi_)) ||
          ((target_theta - curent_theta) > (-pi_)) && ((target_theta - curent_theta) < (0.0)))
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Turn Right :" << (target_theta - curent_theta));
        cmd_vel_data.angular.z = -0.3;
      }
      // Gửi tốc độ xoay cho robot
      cmd_vel_pub->publish(cmd_vel_data);

      // Nếu robot đã đạt được góc quay mong muốn, dừng quay
      if (((std::abs(target_theta - curent_theta) < error_theta)) ||
          (std::abs(target_theta - curent_theta) > (2 * pi_ - error_theta)))
      {
        cmd_vel_data.angular.z = 0.0;
        cmd_vel_pub->publish(cmd_vel_data);
        rclcpp::Rate loop_rate_(10);
        loop_rate_.sleep();

        break;
      }

      next_point_res = 2; // Moving to Posse
      next_point_res_data.data = next_point_res;
      next_point_res_pub->publish(next_point_res_data);

      loop_rate.sleep();
    }
  }
  // Dừng quay nếu không cần thiết
  cmd_vel_data.angular.z = 0.0;
  cmd_vel_pub->publish(cmd_vel_data);

  loop_rate_.sleep();

  RCLCPP_ERROR(this->get_logger(), "Không cần quay nữa");

  clear_all_costmap();

  loop_rate_.sleep();
  // Đặt tốc độ di chuyển thành 0 và gửi yêu cầu di chuyển tới mục tiêu
  // goal
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  // response
  send_goal_options.goal_response_callback =
      std::bind(&MovingPoseSrvServer::goal_response_callback, this, std::placeholders::_1);
  // feedback
  send_goal_options.feedback_callback =
      std::bind(&MovingPoseSrvServer::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  // resuilt
  send_goal_options.result_callback =
      std::bind(&MovingPoseSrvServer::result_callback, this, std::placeholders::_1);

  this->client_ptr_->async_send_goal(goal_data, send_goal_options);
}

void MovingPoseSrvServer::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
{
  // Xử lý khi goal được chấp nhận và từ chối bởi server
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Mục tiêu đã bị máy chủ từ chối");
    next_point_req_ = false;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Mục tiêu đã cháp nhận vởi máy chủ,đợi kq");
    next_point_req_ = true;
  }
}

void MovingPoseSrvServer::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
  // Xử lý kết quả sau khi gửi một goal
  RCLCPP_INFO_STREAM(this->get_logger(), "Kết quả: ");
  next_point_res = 0; // Done to Posse
  next_point_res_data.data = next_point_res;
  next_point_res_pub->publish(next_point_res_data);
}

void MovingPoseSrvServer::cancel_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_cancle)
{
  //// Xử lý khi yêu cầu hủy goal được gửi đến server
  RCLCPP_INFO_STREAM(this->get_logger(), "Hủy goal");
  next_point_req_ = false;
  //  auto cancel_result_future = this->client_ptr_->async_send_goal(goal_cancle);
}

void MovingPoseSrvServer::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // Xử lý phản hồi từ server về quá trình di chuyển
  next_point_res = 2; // Moving to Posse
  next_point_res_data.data = next_point_res;
  next_point_res_pub->publish(next_point_res_data);

  // RCLCPP_INFO_STREAM(this->get_logger(), "Feadd back : ");
}

// void MovingPoseSrvServer::operator_two_ints_callback(const robot_msgs::srv::OperatorTwoInts::Request::SharedPtr req,
//                                                      const robot_msgs::srv::OperatorTwoInts::Response::SharedPtr res)
// {
//   // Xử lý yêu cầu từ service nhận hai số và trả về tổng của chúng
//   res->result = req->a + req->b;
//   std_msgs::msg::Int32 msg;
//   rclcpp::Rate loop_rate_(1);
//   loop_rate_.sleep();

//   msg.data = res->result;
//   _response_pub->publish(msg);
//   RCLCPP_INFO_STREAM(this->get_logger(), " operator_two_ints_callback Completely : ");
// }

void MovingPoseSrvServer::clear_all_costmap()
{
  // Xóa toàn bộ costmap cục bộ và toàn cầu
  auto request_local = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto request_global = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  request_local->request = {};
  request_global->request = {};

  auto result_local = clear_local_cost_map_cl->async_send_request(request_local);
  auto result_global = clear_global_cost_map_cl->async_send_request(request_global);

  RCLCPP_INFO_STREAM(this->get_logger(), " Clear Costmap Completely : ");
}

void MovingPoseSrvServer::navigate_pose_server_cb(const robot_msgs::srv::NavigatePose::Request::SharedPtr req,
                                                  const robot_msgs::srv::NavigatePose::Response::SharedPtr res)
{
  // Xử lý yêu cầu điều hướng đến một vị trí cụ thể
  RCLCPP_INFO_STREAM(this->get_logger(), "Mode :" << req->mode);
  RCLCPP_INFO_STREAM(this->get_logger(), "Pose :" << req->pose[0]);

  float x, y, yaw;

  x = req->pose[0];
  y = req->pose[1];
  yaw = req->pose[2];

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = x;
  goal_pose.pose.position.y = y;
  goal_pose.pose.position.z = 0.0;

  goal_pose.pose.orientation.x = q.x();
  goal_pose.pose.orientation.y = q.y();
  goal_pose.pose.orientation.z = q.z();
  goal_pose.pose.orientation.w = q.w();

  res->result = 1;
  RCLCPP_INFO(this->get_logger(), "x : %f  y :%f  z :%f  w:%f ",
              x, y, q.z(), q.w());

  send_goal_pose();

  RCLCPP_INFO(this->get_logger(), "x : %f  y :%f  z :%f  w:%f ",
              x, y, q.z(), q.w());
}

double MovingPoseSrvServer::quat_to_yaw(geometry_msgs::msg::PoseStamped pose)
{
  auto quat = pose.pose.orientation;
  // Taken from ignition math quaternion Euler()
  double yaw = std::atan2(2 * (quat.x * quat.y + quat.w * quat.z),
                          (quat.w * quat.w) + (quat.x * quat.x) - (quat.y * quat.y) -
                              (quat.z * quat.z));
  return yaw;
}
// Kiểm tra transform từ robot đến mục tiêu
bool MovingPoseSrvServer::get_pose_robot()
{
  geometry_msgs::msg::TransformStamped t;

  try
  {
    t = tf_buffer_->lookupTransform(
        "map", "base_footprint",
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Không thể biến đổi vị trí của trobot");
    return false;
  }

  curent_pose_robot.pose.position.x = t.transform.translation.x;
  curent_pose_robot.pose.position.y = t.transform.translation.y;
  curent_pose_robot.pose.position.z = t.transform.translation.z;

  curent_pose_robot.pose.orientation.x = t.transform.rotation.x;
  curent_pose_robot.pose.orientation.y = t.transform.rotation.y;
  curent_pose_robot.pose.orientation.z = t.transform.rotation.z;
  curent_pose_robot.pose.orientation.w = t.transform.rotation.w;

  return true;
}

void MovingPoseSrvServer::curent_pose_robot_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
}
void MovingPoseSrvServer::next_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

  RCLCPP_ERROR(this->get_logger(), "next_pose_cb");
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = this->get_clock()->now();

  goal_pose.pose.position.x = msg->pose.position.x;
  goal_pose.pose.position.y = msg->pose.position.y;

  goal_pose.pose.orientation.x = msg->pose.orientation.x;
  goal_pose.pose.orientation.y = msg->pose.orientation.y;
  goal_pose.pose.orientation.z = msg->pose.orientation.z;
  goal_pose.pose.orientation.w = msg->pose.orientation.w;

  RCLCPP_INFO(this->get_logger(), "x : %f  y :%f  z :%f  w:%f ",
              goal_pose.pose.position.x, goal_pose.pose.position.y,
              goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);

  next_point_req_ = true;

  send_goal_pose();
}
void MovingPoseSrvServer::robot_node_sleep()
{
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MovingPoseSrvServer>());

  rclcpp::shutdown();

  return 0;
}