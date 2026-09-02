#include "elfin3_console/console_node.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace elfin3_console
{
namespace
{

constexpr std::array<const char *, 6> kExpectedJoints = {
  "elfin_joint1",
  "elfin_joint2",
  "elfin_joint3",
  "elfin_joint4",
  "elfin_joint5",
  "elfin_joint6",
};

constexpr double kRadiansToDegrees =
  180.0 / 3.14159265358979323846;

constexpr std::array<std::array<double, 3>, 3>
kRelativeSequenceNegativeTranslationsCm = {{
  {{0.0, 0.0, 10.0}},
  {{-50.0, 20.0, 0.0}},
  {{0.0, 0.0, -10.0}},
}};

constexpr std::array<std::array<double, 3>, 3>
kRelativeSequencePositiveTranslationsCm = {{
  {{0.0, 0.0, 10.0}},
  {{50.0, -20.0, 0.0}},
  {{0.0, 0.0, -10.0}},
}};

constexpr auto kRelativeSequenceDwell = std::chrono::milliseconds(500);

constexpr std::array<const char *, 6> kSupervisorDiagnosticNames = {
  "elfin3_supervisor/system",
  "elfin3_supervisor/joint_states",
  "elfin3_supervisor/tool_transform",
  "elfin3_supervisor/controllers",
  "elfin3_supervisor/moveit",
  "elfin3_supervisor/clock",
};

const char * diagnosticLevelText(std::uint8_t level)
{
  if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return "OK";
  }
  if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    return "WARN";
  }
  if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    return "ERROR";
  }
  return "STALE";
}

const char * actionResultCodeText(rclcpp_action::ResultCode result_code)
{
  switch (result_code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
  }
}

bool parseFiniteDouble(
  const std::string & text, double & value, std::string & reason)
{
  try {
    std::size_t parsed_characters = 0;
    value = std::stod(text, &parsed_characters);
    if (parsed_characters != text.size() || !std::isfinite(value)) {
      reason = "'" + text + "' is not a finite number";
      return false;
    }
    return true;
  } catch (const std::exception &) {
    reason = "'" + text + "' is not a finite number";
    return false;
  }
}

bool scalingIsValid(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

void requireNonEmpty(const std::string & value, const char * parameter_name)
{
  if (value.empty()) {
    throw std::invalid_argument(std::string(parameter_name) + " must not be empty");
  }
}

void requirePositiveFinite(double value, const char * parameter_name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(
            std::string(parameter_name) + " must be finite and greater than zero");
  }
}

}  // namespace

ConsoleNode::ConsoleNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("elfin3_console", options)
{
  base_frame_ = declare_parameter<std::string>("base_frame", "elfin_base");
  tool_frame_ = declare_parameter<std::string>("tool_frame", "elfin_end_link");
  joint_state_timeout_sec_ =
    declare_parameter<double>("joint_state_timeout_sec", 0.5);
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
  tf_lookup_timeout_sec_ =
    declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
  diagnostics_timeout_sec_ =
    declare_parameter<double>("diagnostics_timeout_sec", 2.0);
  supervisor_timeout_sec_ =
    declare_parameter<double>("supervisor_timeout_sec", 0.5);
  action_server_timeout_sec_ =
    declare_parameter<double>("action_server_timeout_sec", 0.5);
  stop_service_timeout_sec_ =
    declare_parameter<double>("stop_service_timeout_sec", 0.5);
  input_poll_timeout_ms_ =
    declare_parameter<std::int64_t>("input_poll_timeout_ms", 100);
  terminal_output_ = declare_parameter<bool>("terminal_output", true);

  requireNonEmpty(base_frame_, "base_frame");
  requireNonEmpty(tool_frame_, "tool_frame");
  requirePositiveFinite(joint_state_timeout_sec_, "joint_state_timeout_sec");
  requirePositiveFinite(tf_timeout_sec_, "tf_timeout_sec");
  requirePositiveFinite(tf_lookup_timeout_sec_, "tf_lookup_timeout_sec");
  requirePositiveFinite(diagnostics_timeout_sec_, "diagnostics_timeout_sec");
  requirePositiveFinite(supervisor_timeout_sec_, "supervisor_timeout_sec");
  requirePositiveFinite(action_server_timeout_sec_, "action_server_timeout_sec");
  requirePositiveFinite(stop_service_timeout_sec_, "stop_service_timeout_sec");
  if (input_poll_timeout_ms_ <= 0 ||
    input_poll_timeout_ms_ > static_cast<std::int64_t>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("input_poll_timeout_ms is outside the supported range");
  }

  RCLCPP_INFO(
    get_logger(),
    "Elfin3 console skeleton ready: base=%s tool=%s terminal_output=%s",
    base_frame_.c_str(), tool_frame_.c_str(), terminal_output_ ? "true" : "false");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message)
    {
      handleJointState(*message);
    });

  diagnostics_subscription_ =
    create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
    {
      handleDiagnostics(*message);
    });

  supervisor_client_ =
    create_client<SupervisorTrigger>("/elfin3_supervisor/is_ready");
  stop_client_ = create_client<SupervisorTrigger>("/elfin3_motion/stop");
  move_j_client_ = rclcpp_action::create_client<MoveJ>(
    this, "/elfin3_motion/move_j");
  move_pose_client_ = rclcpp_action::create_client<MovePose>(
    this, "/elfin3_motion/move_pose");
  move_pose_ptp_client_ = rclcpp_action::create_client<MovePose>(
    this, "/elfin3_motion/move_pose_ptp");
  move_pose_relative_sequence_callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_update_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), [this]() {updateToolPose();});

  RCLCPP_INFO(
    get_logger(),
    "Monitoring /joint_states, %s -> %s TF, and supervisor diagnostics",
    base_frame_.c_str(), tool_frame_.c_str());
}

ConsoleNode::~ConsoleNode()
{
  prepareForShutdown();
  if (input_thread_.joinable()) {
    input_thread_.join();
  }
}

void ConsoleNode::start()
{
  if (input_started_) {
    throw std::logic_error("console input is already started");
  }
  input_started_ = true;

  if (!terminal_output_) {
    RCLCPP_INFO(get_logger(), "Terminal input is disabled by terminal_output=false");
    return;
  }

  input_stop_requested_.store(false);
  input_thread_ = std::thread([this]() {inputLoop();});
}

void ConsoleNode::prepareForShutdown()
{
  input_stop_requested_.store(true);
  cancelActiveGoalForShutdown();
}

void ConsoleNode::handleJointState(const sensor_msgs::msg::JointState & message)
{
  if (message.name.size() != message.position.size()) {
    std::ostringstream reason;
    reason << "name size (" << message.name.size() << ") differs from position size (" <<
      message.position.size() << ')';
    rejectJointState(JointStateHealth::kInvalid, reason.str());
    return;
  }

  std::unordered_map<std::string, std::size_t> name_to_index;
  name_to_index.reserve(message.name.size());
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    if (!name_to_index.emplace(message.name[index], index).second) {
      rejectJointState(
        JointStateHealth::kInvalid,
        "duplicate joint name '" + message.name[index] + "'");
      return;
    }
  }

  std::array<double, kJointCount> ordered_positions{};
  for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
    const auto found = name_to_index.find(kExpectedJoints[joint_index]);
    if (found == name_to_index.end()) {
      rejectJointState(
        JointStateHealth::kIncomplete,
        "missing expected joint '" + std::string(kExpectedJoints[joint_index]) + "'");
      return;
    }

    const double position = message.position[found->second];
    if (!std::isfinite(position)) {
      rejectJointState(
        JointStateHealth::kInvalid,
        "non-finite position for '" + std::string(kExpectedJoints[joint_index]) + "'");
      return;
    }
    ordered_positions[joint_index] = position;
  }

  bool first_valid_sample = false;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    first_valid_sample = !has_valid_joint_state_;
    latest_joint_positions_ = ordered_positions;
    last_valid_joint_state_time_ = std::chrono::steady_clock::now();
    joint_state_health_ = JointStateHealth::kValid;
    joint_state_issue_.clear();
    has_valid_joint_state_ = true;
  }

  if (first_valid_sample) {
    RCLCPP_INFO(get_logger(), "First complete six-axis JointState received");
  }
}

void ConsoleNode::rejectJointState(
  JointStateHealth health, const std::string & reason)
{
  bool issue_changed = false;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    issue_changed = joint_state_health_ != health || joint_state_issue_ != reason;
    joint_state_health_ = health;
    joint_state_issue_ = reason;
  }

  if (issue_changed) {
    RCLCPP_WARN(get_logger(), "Rejected JointState: %s", reason.c_str());
  }
}

void ConsoleNode::handleDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray & message)
{
  bool supervisor_status_seen = false;
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  for (const auto & status : message.status) {
    if (status.name.compare(0, 18, "elfin3_supervisor/") != 0) {
      continue;
    }

    supervisor_diagnostics_[status.name] = {
      diagnosticLevelText(status.level), status.message};
    if (status.name == "elfin3_supervisor/system") {
      supervisor_status_seen = true;
      supervisor_reason_ = status.message;
      supervisor_state_ = "UNKNOWN";
      for (const auto & value : status.values) {
        if (value.key == "state") {
          supervisor_state_ = value.value;
          break;
        }
      }
    }
  }

  if (supervisor_status_seen) {
    has_supervisor_diagnostics_ = true;
    last_diagnostics_time_ = std::chrono::steady_clock::now();
  }
}

void ConsoleNode::updateToolPose()
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      base_frame_, tool_frame_, tf2::TimePointZero,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    const bool values_are_finite =
      std::isfinite(translation.x) && std::isfinite(translation.y) &&
      std::isfinite(translation.z) && std::isfinite(rotation.x) &&
      std::isfinite(rotation.y) && std::isfinite(rotation.z) &&
      std::isfinite(rotation.w);
    const double quaternion_norm = std::sqrt(
      rotation.x * rotation.x + rotation.y * rotation.y +
      rotation.z * rotation.z + rotation.w * rotation.w);

    std::lock_guard<std::mutex> lock(tf_mutex_);
    if (!values_are_finite || !std::isfinite(quaternion_norm) ||
      quaternion_norm <= 1.0e-6)
    {
      tf_available_ = true;
      tf_values_valid_ = false;
      tf_issue_ = "transform contains invalid values";
      return;
    }

    const std::int64_t stamp_ns =
      static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(transform.header.stamp.nanosec);
    const auto steady_now = std::chrono::steady_clock::now();
    if (!tf_available_) {
      last_tf_stamp_ns_ = stamp_ns;
      last_tf_update_time_ = steady_now;
      tf_time_reset_ = false;
    } else if (stamp_ns > last_tf_stamp_ns_) {
      last_tf_stamp_ns_ = stamp_ns;
      last_tf_update_time_ = steady_now;
      tf_time_reset_ = false;
    } else if (stamp_ns < last_tf_stamp_ns_) {
      last_tf_stamp_ns_ = stamp_ns;
      last_tf_update_time_ = steady_now;
      tf_time_reset_ = true;
    }

    latest_tool_position_ = {translation.x, translation.y, translation.z};
    latest_tool_orientation_ = {
      rotation.x / quaternion_norm,
      rotation.y / quaternion_norm,
      rotation.z / quaternion_norm,
      rotation.w / quaternion_norm,
    };
    tf_available_ = true;
    tf_values_valid_ = true;
    tf_issue_.clear();
  } catch (const tf2::TransformException & exception) {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    tf_available_ = false;
    tf_values_valid_ = false;
    tf_issue_ = exception.what();
  }
}

void ConsoleNode::displayStatus()
{
  if (!terminal_output_) {
    return;
  }

  std::array<double, kJointCount> positions{};
  std::chrono::steady_clock::time_point receive_time{};
  JointStateHealth health = JointStateHealth::kNoData;
  std::string issue;
  bool has_valid_sample = false;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    positions = latest_joint_positions_;
    receive_time = last_valid_joint_state_time_;
    health = joint_state_health_;
    issue = joint_state_issue_;
    has_valid_sample = has_valid_joint_state_;
  }

  std::array<double, 3> tool_position{};
  std::array<double, 4> tool_orientation{};
  std::chrono::steady_clock::time_point tf_update_time{};
  std::string tf_issue;
  bool tf_available = false;
  bool tf_values_valid = false;
  bool tf_time_reset = false;
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    tool_position = latest_tool_position_;
    tool_orientation = latest_tool_orientation_;
    tf_update_time = last_tf_update_time_;
    tf_issue = tf_issue_;
    tf_available = tf_available_;
    tf_values_valid = tf_values_valid_;
    tf_time_reset = tf_time_reset_;
  }

  std::unordered_map<std::string, DiagnosticLine> diagnostics;
  std::chrono::steady_clock::time_point diagnostics_time{};
  std::string supervisor_state;
  std::string supervisor_reason;
  bool has_diagnostics = false;
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    diagnostics = supervisor_diagnostics_;
    diagnostics_time = last_diagnostics_time_;
    supervisor_state = supervisor_state_;
    supervisor_reason = supervisor_reason_;
    has_diagnostics = has_supervisor_diagnostics_;
  }

  const auto steady_now = std::chrono::steady_clock::now();
  const bool diagnostics_fresh = has_diagnostics &&
    std::chrono::duration<double>(steady_now - diagnostics_time).count() <=
    diagnostics_timeout_sec_;
  const bool tf_fresh = tf_available && tf_values_valid && !tf_time_reset &&
    std::chrono::duration<double>(steady_now - tf_update_time).count() <=
    tf_timeout_sec_;

  std::ostringstream output;
  output << "Elfin3 Console Monitor\n";
  output << "System: " << (diagnostics_fresh ? supervisor_state : "UNKNOWN") <<
    "  Reason: " << (diagnostics_fresh ? supervisor_reason : "diagnostics unavailable or stale");

  output << "\nRuntime:";
  for (const auto * name : kSupervisorDiagnosticNames) {
    const auto found = diagnostics.find(name);
    const std::string short_name = std::string(name).substr(18);
    output << "\n  " << std::left << std::setw(14) << short_name << " " <<
      (diagnostics_fresh && found != diagnostics.end() ? found->second.level : "UNKNOWN");
    if (diagnostics_fresh && found != diagnostics.end()) {
      output << " - " << found->second.message;
    }
  }

  output << "\nJointState ";

  if (!has_valid_sample) {
    const std::string detail = issue.empty() ?
      "no complete sample received" : issue;
    output << "WAITING - " << detail;
  } else {
    const double age_sec = std::chrono::duration<double>(
      steady_now - receive_time).count();
    const bool fresh = age_sec <= joint_state_timeout_sec_;
    const char * status = !fresh ? "STALE" :
      (health == JointStateHealth::kValid ? "OK" : "INPUT_REJECTED");
    output << status << " age=" << std::fixed << std::setprecision(3) << age_sec << " s";
    if (health != JointStateHealth::kValid && !issue.empty()) {
      output << " latest_input='" << issue << '\'';
    }
    output << '\n' << "Joint    rad          deg";
    output << std::right << std::setprecision(6);
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
      output << '\n' << "J" << (joint_index + 1) << "    " <<
        std::setw(10) << positions[joint_index] << "   " <<
        std::setw(10) << positions[joint_index] * kRadiansToDegrees;
    }
  }

  output << "\nTCP [" << base_frame_ << " -> " << tool_frame_ << "] ";
  if (!tf_fresh) {
    output << "UNAVAILABLE";
    if (!tf_issue.empty()) {
      output << " - " << tf_issue;
    }
  } else {
    output << "OK\nXYZ [m]  x=" << std::fixed << std::setprecision(6) <<
      tool_position[0] << " y=" << tool_position[1] << " z=" << tool_position[2];
    output << "\nQuaternion [xyzw]  x=" << tool_orientation[0] <<
      " y=" << tool_orientation[1] << " z=" << tool_orientation[2] <<
      " w=" << tool_orientation[3];
  }

  printOutput(output.str());
}

void ConsoleNode::displayJointState()
{
  std::array<double, kJointCount> positions{};
  std::chrono::steady_clock::time_point receive_time{};
  JointStateHealth health = JointStateHealth::kNoData;
  std::string issue;
  bool has_valid_sample = false;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    positions = latest_joint_positions_;
    receive_time = last_valid_joint_state_time_;
    health = joint_state_health_;
    issue = joint_state_issue_;
    has_valid_sample = has_valid_joint_state_;
  }

  std::ostringstream output;
  if (!has_valid_sample) {
    output << "JointState WAITING - " <<
      (issue.empty() ? "no complete sample received" : issue);
    printOutput(output.str());
    return;
  }

  const double age_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - receive_time).count();
  const bool fresh = age_sec <= joint_state_timeout_sec_;
  const char * status = !fresh ? "STALE" :
    (health == JointStateHealth::kValid ? "OK" : "INPUT_REJECTED");
  output << "JointState " << status << " age=" <<
    std::fixed << std::setprecision(3) << age_sec << " s";
  if (health != JointStateHealth::kValid && !issue.empty()) {
    output << " latest_input='" << issue << '\'';
  }
  output << '\n' << "Joint    rad          deg" <<
    std::right << std::setprecision(6);
  for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
    output << '\n' << "J" << (joint_index + 1) << "    " <<
      std::setw(10) << positions[joint_index] << "   " <<
      std::setw(10) << positions[joint_index] * kRadiansToDegrees;
  }
  printOutput(output.str());
}

void ConsoleNode::displayToolPose()
{
  updateToolPose();

  std::array<double, 3> tool_position{};
  std::array<double, 4> tool_orientation{};
  std::chrono::steady_clock::time_point update_time{};
  std::string issue;
  bool available = false;
  bool values_valid = false;
  bool time_reset = false;
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    tool_position = latest_tool_position_;
    tool_orientation = latest_tool_orientation_;
    update_time = last_tf_update_time_;
    issue = tf_issue_;
    available = tf_available_;
    values_valid = tf_values_valid_;
    time_reset = tf_time_reset_;
  }

  const bool fresh = available && values_valid && !time_reset &&
    std::chrono::duration<double>(
    std::chrono::steady_clock::now() - update_time).count() <= tf_timeout_sec_;

  std::ostringstream output;
  output << "TCP [" << base_frame_ << " -> " << tool_frame_ << "] ";
  if (!fresh) {
    output << "UNAVAILABLE";
    if (!issue.empty()) {
      output << " - " << issue;
    }
    printOutput(output.str());
    return;
  }

  output << "OK\nXYZ [m]  x=" << std::fixed << std::setprecision(6) <<
    tool_position[0] << " y=" << tool_position[1] << " z=" << tool_position[2];
  output << "\nQuaternion [xyzw]  x=" << tool_orientation[0] <<
    " y=" << tool_orientation[1] << " z=" << tool_orientation[2] <<
    " w=" << tool_orientation[3];
  printOutput(output.str());
}

void ConsoleNode::displayHelp()
{
  printOutput(
    "Commands:\n"
    "  help    Show this command list\n"
    "  status  Show system, runtime, joints, and TCP\n"
    "  joints  Show J1-to-J6 positions in rad and deg\n"
    "  tcp     Show elfin_end_link pose in elfin_base\n"
    "  movej J1 J2 J3 J4 J5 J6 [velocity [acceleration]]\n"
    "  movep X Y Z QX QY QZ QW [velocity [acceleration]]\n"
    "  moveptp X Y Z QX QY QZ QW [velocity [acceleration]]\n"
    "  moveptpr DX_CM DY_CM DZ_CM [velocity [acceleration]]\n"
    "           Base-frame relative PTP translation in cm; keep current TCP orientation\n"
    "           Endpoint target only; motion is planned once in joint space\n"
    "  moveptprseq [velocity [acceleration]]\n"
    "           +Z 10 cm, wait 0.5 s, -X 50/+Y 20 cm, wait 0.5 s, -Z 10 cm\n"
    "           Steps 2-3 force TCP vertically down while preserving current yaw\n"
    "  moveptprseqpos [velocity [acceleration]]\n"
    "           +Z 10 cm, wait 0.5 s, +X 50/-Y 20 cm, wait 0.5 s, -Z 10 cm\n"
    "           Steps 2-3 force TCP vertically down while preserving current yaw\n"
    "  cancel  Cancel the motion goal sent by this console\n"
    "  stop    Request the motion command module to stop\n"
    "  quit    Exit when no local motion command is active");
}

void ConsoleNode::handleCommand(const std::string & line)
{
  std::istringstream input(line);
  std::string command;
  if (!(input >> command)) {
    printPrompt();
    return;
  }

  std::vector<std::string> arguments;
  std::string argument;
  while (input >> argument) {
    arguments.push_back(argument);
  }

  if (command == "movej") {
    handleMoveJCommand(arguments);
    return;
  }
  if (command == "movep") {
    handleMovePoseCommand(arguments, false);
    return;
  }
  if (command == "moveptp") {
    handleMovePoseCommand(arguments, true);
    return;
  }
  if (command == "moveptpr") {
    handleMovePoseRelativeCommand(arguments);
    return;
  }
  if (command == "moveptprseq") {
    handleMovePoseRelativeSequenceCommand(arguments, false);
    return;
  }
  if (command == "moveptprseqpos") {
    handleMovePoseRelativeSequenceCommand(arguments, true);
    return;
  }

  if (!arguments.empty()) {
    printOutput("ERROR: command '" + command + "' does not accept arguments");
    return;
  }

  if (command == "help") {
    displayHelp();
  } else if (command == "status") {
    updateToolPose();
    displayStatus();
  } else if (command == "joints") {
    displayJointState();
  } else if (command == "tcp") {
    displayToolPose();
  } else if (command == "cancel") {
    handleCancelCommand();
  } else if (command == "stop") {
    handleStopCommand();
  } else if (command == "quit") {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_state_ != CommandState::kIdle) {
        printOutput("BUSY: use cancel or stop before quitting");
        return;
      }
    }
    printOutput("Exiting elfin3_console", false);
    input_stop_requested_.store(true);
    rclcpp::shutdown();
  } else {
    printOutput("ERROR: unknown command '" + command + "'; type 'help'");
  }
}

void ConsoleNode::handleMoveJCommand(
  const std::vector<std::string> & arguments)
{
  if (arguments.size() < kJointCount || arguments.size() > kJointCount + 2) {
    printOutput(
      "ERROR: usage: movej J1 J2 J3 J4 J5 J6 "
      "[velocity_scaling [acceleration_scaling]]");
    return;
  }

  MoveJ::Goal goal;
  std::string reason;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!parseFiniteDouble(arguments[index], goal.joint_positions[index], reason)) {
      printOutput("ERROR: J" + std::to_string(index + 1) + ": " + reason);
      return;
    }
  }

  goal.velocity_scaling = 0.0;
  goal.acceleration_scaling = 0.0;
  if (arguments.size() >= kJointCount + 1 &&
    !parseFiniteDouble(arguments[kJointCount], goal.velocity_scaling, reason))
  {
    printOutput("ERROR: velocity_scaling: " + reason);
    return;
  }
  if (arguments.size() == kJointCount + 2 &&
    !parseFiniteDouble(arguments[kJointCount + 1], goal.acceleration_scaling, reason))
  {
    printOutput("ERROR: acceleration_scaling: " + reason);
    return;
  }
  if (!scalingIsValid(goal.velocity_scaling)) {
    printOutput("ERROR: velocity_scaling must be within [0.0, 1.0]");
    return;
  }
  if (!scalingIsValid(goal.acceleration_scaling)) {
    printOutput("ERROR: acceleration_scaling must be within [0.0, 1.0]");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutdown_cleanup_started_.load()) {
      printOutput("ERROR: console shutdown is already in progress");
      return;
    }
    if (command_state_ != CommandState::kIdle) {
      printOutput("BUSY: another local motion command is active");
      return;
    }
  }

  if (!querySupervisorReadiness(reason)) {
    printOutput("NOT_READY: " + reason);
    return;
  }

  const auto action_timeout =
    std::chrono::duration<double>(action_server_timeout_sec_);
  if (!move_j_client_->wait_for_action_server(action_timeout)) {
    printOutput("NOT_READY: /elfin3_motion/move_j is unavailable");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutdown_cleanup_started_.load()) {
      printOutput("ERROR: console shutdown is already in progress");
      return;
    }
    if (command_state_ != CommandState::kIdle) {
      printOutput("BUSY: another local motion command became active");
      return;
    }
    command_state_ = CommandState::kSending;
    command_type_ = CommandType::kMoveJ;
    cancel_when_accepted_ = false;
  }

  rclcpp_action::Client<MoveJ>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const MoveJGoalHandle::SharedPtr & goal_handle)
    {
      handleMoveJGoalResponse(goal_handle);
    };
  options.feedback_callback =
    [this](
      MoveJGoalHandle::SharedPtr goal_handle,
      const std::shared_ptr<const MoveJ::Feedback> feedback)
    {
      handleMoveJFeedback(goal_handle, feedback);
    };
  options.result_callback =
    [this](const MoveJGoalHandle::WrappedResult & wrapped_result)
    {
      handleMoveJResult(wrapped_result);
    };

  try {
    move_j_client_->async_send_goal(goal, options);
    printOutput("MoveJ goal submitted: state=SENDING");
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      cancel_when_accepted_ = false;
    }
    printOutput(std::string("ERROR: failed to send MoveJ goal: ") + exception.what());
  }
}

bool ConsoleNode::handleMovePoseCommand(
  const std::vector<std::string> & arguments,
  const bool point_to_point,
  const bool sequence_step)
{
  const std::string command_name = sequence_step ?
    "MovePosePTPSequence" : (point_to_point ? "MovePosePTP" : "MovePose");
  const std::string command_keyword = point_to_point ? "moveptp" : "movep";
  const std::string action_name = point_to_point ?
    "/elfin3_motion/move_pose_ptp" : "/elfin3_motion/move_pose";
  const auto action_client = point_to_point ? move_pose_ptp_client_ : move_pose_client_;
  constexpr std::size_t pose_component_count = 7;
  if (arguments.size() < pose_component_count ||
    arguments.size() > pose_component_count + 2)
  {
    printOutput(
      "ERROR: usage: " + command_keyword +
      " X Y Z QX QY QZ QW [velocity_scaling [acceleration_scaling]]");
    return false;
  }

  std::array<double, pose_component_count> components{};
  std::string reason;
  for (std::size_t index = 0; index < pose_component_count; ++index) {
    if (!parseFiniteDouble(arguments[index], components[index], reason)) {
      printOutput(
        "ERROR: pose component " + std::to_string(index + 1) + ": " + reason);
      return false;
    }
  }

  const double quaternion_norm = std::sqrt(
    components[3] * components[3] + components[4] * components[4] +
    components[5] * components[5] + components[6] * components[6]);
  if (!std::isfinite(quaternion_norm) || quaternion_norm <= 1.0e-6) {
    printOutput("ERROR: quaternion norm must be greater than 1e-6");
    return false;
  }

  MovePose::Goal goal;
  goal.target_pose.header.frame_id = base_frame_;
  goal.target_pose.pose.position.x = components[0];
  goal.target_pose.pose.position.y = components[1];
  goal.target_pose.pose.position.z = components[2];
  goal.target_pose.pose.orientation.x = components[3];
  goal.target_pose.pose.orientation.y = components[4];
  goal.target_pose.pose.orientation.z = components[5];
  goal.target_pose.pose.orientation.w = components[6];
  goal.velocity_scaling = 0.0;
  goal.acceleration_scaling = 0.0;

  if (arguments.size() >= pose_component_count + 1 &&
    !parseFiniteDouble(
      arguments[pose_component_count], goal.velocity_scaling, reason))
  {
    printOutput("ERROR: velocity_scaling: " + reason);
    return false;
  }
  if (arguments.size() == pose_component_count + 2 &&
    !parseFiniteDouble(
      arguments[pose_component_count + 1], goal.acceleration_scaling, reason))
  {
    printOutput("ERROR: acceleration_scaling: " + reason);
    return false;
  }
  if (!scalingIsValid(goal.velocity_scaling)) {
    printOutput("ERROR: velocity_scaling must be within [0.0, 1.0]");
    return false;
  }
  if (!scalingIsValid(goal.acceleration_scaling)) {
    printOutput("ERROR: acceleration_scaling must be within [0.0, 1.0]");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutdown_cleanup_started_.load()) {
      printOutput("ERROR: console shutdown is already in progress");
      return false;
    }
    const bool command_slot_available = sequence_step ?
      (move_pose_relative_sequence_active_ &&
      command_state_ == CommandState::kWaiting) :
      command_state_ == CommandState::kIdle;
    if (!command_slot_available) {
      printOutput("BUSY: another local motion command is active");
      return false;
    }
  }

  if (!querySupervisorReadiness(reason)) {
    printOutput("NOT_READY: " + reason);
    return false;
  }

  const auto action_timeout =
    std::chrono::duration<double>(action_server_timeout_sec_);
  if (!action_client->wait_for_action_server(action_timeout)) {
    printOutput("NOT_READY: " + action_name + " is unavailable");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutdown_cleanup_started_.load()) {
      printOutput("ERROR: console shutdown is already in progress");
      return false;
    }
    const bool command_slot_available = sequence_step ?
      (move_pose_relative_sequence_active_ &&
      command_state_ == CommandState::kWaiting) :
      command_state_ == CommandState::kIdle;
    if (!command_slot_available) {
      printOutput("BUSY: another local motion command became active");
      return false;
    }
    command_state_ = CommandState::kSending;
    command_type_ = CommandType::kMovePose;
    cancel_when_accepted_ = false;
    active_move_pose_client_ = action_client;
    active_move_pose_name_ = command_name;
  }

  rclcpp_action::Client<MovePose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const MovePoseGoalHandle::SharedPtr & goal_handle)
    {
      handleMovePoseGoalResponse(goal_handle);
    };
  options.feedback_callback =
    [this](
      MovePoseGoalHandle::SharedPtr goal_handle,
      const std::shared_ptr<const MovePose::Feedback> feedback)
    {
      handleMovePoseFeedback(goal_handle, feedback);
    };
  options.result_callback =
    [this](const MovePoseGoalHandle::WrappedResult & wrapped_result)
    {
      handleMovePoseResult(wrapped_result);
    };

  try {
    action_client->async_send_goal(goal, options);
    printOutput(command_name + " goal submitted: state=SENDING");
    return true;
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_state_ = sequence_step ?
        CommandState::kWaiting : CommandState::kIdle;
      command_type_ = CommandType::kNone;
      cancel_when_accepted_ = false;
      active_move_pose_client_.reset();
      active_move_pose_name_ = "MovePose";
    }
    printOutput(
      std::string("ERROR: failed to send ") + command_name + " goal: " + exception.what());
    return false;
  }
}

bool ConsoleNode::handleMovePoseRelativeCommand(
  const std::vector<std::string> & arguments,
  const bool sequence_step,
  const bool downward_orientation)
{
  constexpr std::size_t translation_component_count = 3;
  constexpr double centimeters_to_meters = 0.01;
  if (arguments.size() < translation_component_count ||
    arguments.size() > translation_component_count + 2)
  {
    printOutput(
      "ERROR: usage: moveptpr DX_CM DY_CM DZ_CM "
      "[velocity_scaling [acceleration_scaling]]");
    return false;
  }

  std::array<double, translation_component_count> translation_cm{};
  std::string reason;
  for (std::size_t index = 0; index < translation_component_count; ++index) {
    if (!parseFiniteDouble(arguments[index], translation_cm[index], reason)) {
      printOutput(
        "ERROR: relative translation component " + std::to_string(index + 1) +
        ": " + reason);
      return false;
    }
  }

  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  if (arguments.size() >= translation_component_count + 1 &&
    !parseFiniteDouble(
      arguments[translation_component_count], velocity_scaling, reason))
  {
    printOutput("ERROR: velocity_scaling: " + reason);
    return false;
  }
  if (arguments.size() == translation_component_count + 2 &&
    !parseFiniteDouble(
      arguments[translation_component_count + 1], acceleration_scaling, reason))
  {
    printOutput("ERROR: acceleration_scaling: " + reason);
    return false;
  }
  if (!scalingIsValid(velocity_scaling)) {
    printOutput("ERROR: velocity_scaling must be within [0.0, 1.0]");
    return false;
  }
  if (!scalingIsValid(acceleration_scaling)) {
    printOutput("ERROR: acceleration_scaling must be within [0.0, 1.0]");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutdown_cleanup_started_.load()) {
      printOutput("ERROR: console shutdown is already in progress");
      return false;
    }
    const bool command_slot_available = sequence_step ?
      (move_pose_relative_sequence_active_ &&
      command_state_ == CommandState::kWaiting) :
      command_state_ == CommandState::kIdle;
    if (!command_slot_available) {
      printOutput("BUSY: another local motion command is active");
      return false;
    }
  }

  updateToolPose();

  std::array<double, 3> current_position{};
  std::array<double, 4> current_orientation{};
  std::chrono::steady_clock::time_point update_time{};
  std::string tf_issue;
  bool available = false;
  bool values_valid = false;
  bool time_reset = false;
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    current_position = latest_tool_position_;
    current_orientation = latest_tool_orientation_;
    update_time = last_tf_update_time_;
    tf_issue = tf_issue_;
    available = tf_available_;
    values_valid = tf_values_valid_;
    time_reset = tf_time_reset_;
  }

  const auto now = std::chrono::steady_clock::now();
  const double tf_age_sec = available ?
    std::chrono::duration<double>(now - update_time).count() : 0.0;
  const bool fresh = available && values_valid && !time_reset &&
    tf_age_sec <= tf_timeout_sec_;
  if (!fresh) {
    std::ostringstream error;
    error << "NOT_READY: cannot resolve moveptpr from " << base_frame_ << " -> " <<
      tool_frame_ << " TF: ";
    if (!available) {
      error << "transform is unavailable";
    } else if (!values_valid) {
      error << "transform contains invalid values";
    } else if (time_reset) {
      error << "transform timestamp moved backwards";
    } else {
      error << "transform is stale (age=" << std::fixed << std::setprecision(3) <<
        tf_age_sec << " s, limit=" << tf_timeout_sec_ << " s)";
    }
    if (!tf_issue.empty()) {
      error << "; " << tf_issue;
    }
    printOutput(error.str());
    return false;
  }

  std::array<double, 3> target_position{};
  for (std::size_t index = 0; index < translation_component_count; ++index) {
    target_position[index] =
      current_position[index] + translation_cm[index] * centimeters_to_meters;
  }

  std::array<double, 4> target_orientation = current_orientation;
  if (downward_orientation) {
    const double yaw = std::atan2(
      2.0 * (
        current_orientation[3] * current_orientation[2] +
        current_orientation[0] * current_orientation[1]),
      1.0 - 2.0 * (
        current_orientation[1] * current_orientation[1] +
        current_orientation[2] * current_orientation[2]));
    target_orientation = {
      std::cos(0.5 * yaw),
      std::sin(0.5 * yaw),
      0.0,
      0.0,
    };
  }

  std::ostringstream resolved;
  resolved << std::fixed << std::setprecision(6) <<
    "MovePosePTPRelative resolved in " << base_frame_ <<
    ": start_xyz=[" << current_position[0] << ", " << current_position[1] <<
    ", " << current_position[2] << "] delta_cm=[" << translation_cm[0] <<
    ", " << translation_cm[1] << ", " << translation_cm[2] <<
    "] target_xyz=[" << target_position[0] << ", " << target_position[1] <<
    ", " << target_position[2] << "] orientation=" <<
    (downward_orientation ? "DOWN_KEEP_YAW" : "KEEP") << " q_xyzw=[" <<
    target_orientation[0] << ", " << target_orientation[1] << ", " <<
    target_orientation[2] << ", " << target_orientation[3] << "]";
  printOutput(resolved.str());

  std::vector<std::string> absolute_arguments;
  absolute_arguments.reserve(9);
  const auto append_double =
    [&absolute_arguments](const double value)
    {
      std::ostringstream stream;
      stream << std::setprecision(17) << value;
      absolute_arguments.push_back(stream.str());
    };
  append_double(target_position[0]);
  append_double(target_position[1]);
  append_double(target_position[2]);
  append_double(target_orientation[0]);
  append_double(target_orientation[1]);
  append_double(target_orientation[2]);
  append_double(target_orientation[3]);
  if (arguments.size() >= translation_component_count + 1) {
    append_double(velocity_scaling);
  }
  if (arguments.size() == translation_component_count + 2) {
    append_double(acceleration_scaling);
  }

  return handleMovePoseCommand(absolute_arguments, true, sequence_step);
}

void ConsoleNode::handleMovePoseRelativeSequenceCommand(
  const std::vector<std::string> & arguments,
  const bool positive_xy)
{
  const std::string command_keyword =
    positive_xy ? "moveptprseqpos" : "moveptprseq";
  if (arguments.size() > 2) {
    printOutput(
      "ERROR: usage: " + command_keyword +
      " [velocity_scaling [acceleration_scaling]]");
    return;
  }

  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  std::string reason;
  if (!arguments.empty() &&
    !parseFiniteDouble(arguments[0], velocity_scaling, reason))
  {
    printOutput("ERROR: velocity_scaling: " + reason);
    return;
  }
  if (arguments.size() == 2 &&
    !parseFiniteDouble(arguments[1], acceleration_scaling, reason))
  {
    printOutput("ERROR: acceleration_scaling: " + reason);
    return;
  }
  if (!scalingIsValid(velocity_scaling)) {
    printOutput("ERROR: velocity_scaling must be within [0.0, 1.0]");
    return;
  }
  if (!scalingIsValid(acceleration_scaling)) {
    printOutput("ERROR: acceleration_scaling must be within [0.0, 1.0]");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutdown_cleanup_started_.load()) {
      printOutput("ERROR: console shutdown is already in progress");
      return;
    }
    if (command_state_ != CommandState::kIdle ||
      move_pose_relative_sequence_active_)
    {
      printOutput("BUSY: another local motion command is active");
      return;
    }
    if (move_pose_relative_sequence_timer_) {
      move_pose_relative_sequence_timer_->cancel();
      move_pose_relative_sequence_timer_.reset();
    }
    move_pose_relative_sequence_active_ = true;
    move_pose_relative_sequence_positive_xy_ = positive_xy;
    move_pose_relative_sequence_step_ = 0;
    move_pose_relative_sequence_velocity_scaling_ = velocity_scaling;
    move_pose_relative_sequence_acceleration_scaling_ = acceleration_scaling;
    command_state_ = CommandState::kWaiting;
    command_type_ = CommandType::kNone;
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(3) <<
    "MovePosePTP sequence started: profile=" <<
    (positive_xy ? "POSITIVE_XY" : "NEGATIVE_XY") <<
    " steps=3 velocity_scaling=" <<
    velocity_scaling << " acceleration_scaling=" << acceleration_scaling;
  printOutput(output.str());
  startMovePoseRelativeSequenceStep();
}

void ConsoleNode::startMovePoseRelativeSequenceStep()
{
  std::size_t step = 0;
  bool positive_xy = false;
  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!move_pose_relative_sequence_active_ ||
      command_state_ != CommandState::kWaiting ||
      move_pose_relative_sequence_step_ >=
      kRelativeSequenceNegativeTranslationsCm.size())
    {
      return;
    }
    step = move_pose_relative_sequence_step_;
    positive_xy = move_pose_relative_sequence_positive_xy_;
    velocity_scaling = move_pose_relative_sequence_velocity_scaling_;
    acceleration_scaling = move_pose_relative_sequence_acceleration_scaling_;
  }

  const auto & translations = positive_xy ?
    kRelativeSequencePositiveTranslationsCm :
    kRelativeSequenceNegativeTranslationsCm;
  const auto & translation = translations[step];
  const bool downward_orientation = step > 0;
  std::ostringstream output;
  output << std::fixed << std::setprecision(3) <<
    "MovePosePTP sequence step " << (step + 1) << "/" <<
    translations.size() << ": delta_cm=[" <<
    translation[0] << ", " << translation[1] << ", " << translation[2] <<
    "] orientation=" <<
    (downward_orientation ? "DOWN_KEEP_YAW" : "KEEP");
  printOutput(output.str());

  std::vector<std::string> arguments;
  arguments.reserve(5);
  const auto append_double =
    [&arguments](const double value)
    {
      std::ostringstream stream;
      stream << std::setprecision(17) << value;
      arguments.push_back(stream.str());
    };
  append_double(translation[0]);
  append_double(translation[1]);
  append_double(translation[2]);
  append_double(velocity_scaling);
  append_double(acceleration_scaling);

  if (handleMovePoseRelativeCommand(arguments, true, downward_orientation)) {
    return;
  }

  bool aborted = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (move_pose_relative_sequence_active_ &&
      move_pose_relative_sequence_step_ == step)
    {
      move_pose_relative_sequence_active_ = false;
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      aborted = true;
    }
  }
  if (aborted) {
    printOutput(
      "MovePosePTP sequence aborted: step " + std::to_string(step + 1) +
      " could not be submitted");
  }
}

void ConsoleNode::scheduleMovePoseRelativeSequenceStep()
{
  std::size_t next_step = 0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!move_pose_relative_sequence_active_ ||
      command_state_ != CommandState::kWaiting ||
      move_pose_relative_sequence_step_ >=
      kRelativeSequenceNegativeTranslationsCm.size())
    {
      return;
    }
    next_step = move_pose_relative_sequence_step_;
  }

  printOutput(
    "MovePosePTP sequence waiting 0.500 s before step " +
    std::to_string(next_step + 1));

  auto timer = create_wall_timer(
    kRelativeSequenceDwell,
    [this]()
    {
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!move_pose_relative_sequence_active_ ||
          command_state_ != CommandState::kWaiting)
        {
          return;
        }
        if (move_pose_relative_sequence_timer_) {
          move_pose_relative_sequence_timer_->cancel();
        }
      }
      startMovePoseRelativeSequenceStep();
    },
    move_pose_relative_sequence_callback_group_);

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (move_pose_relative_sequence_active_ &&
      command_state_ == CommandState::kWaiting)
    {
      move_pose_relative_sequence_timer_ = timer;
    } else {
      timer->cancel();
    }
  }
}

void ConsoleNode::handleCancelCommand()
{
  MoveJGoalHandle::SharedPtr move_j_goal;
  MovePoseGoalHandle::SharedPtr move_pose_goal;
  CommandType command_type = CommandType::kNone;
  std::string immediate_message;

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_state_ == CommandState::kIdle) {
      immediate_message = "NO_ACTIVE_GOAL: no console-owned motion command";
    } else if (command_state_ == CommandState::kWaiting &&
      move_pose_relative_sequence_active_)
    {
      move_pose_relative_sequence_active_ = false;
      if (move_pose_relative_sequence_timer_) {
        move_pose_relative_sequence_timer_->cancel();
        move_pose_relative_sequence_timer_.reset();
      }
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      cancel_when_accepted_ = false;
      immediate_message = "MovePosePTP sequence canceled during dwell";
    } else if (command_state_ == CommandState::kSending) {
      if (move_pose_relative_sequence_active_) {
        move_pose_relative_sequence_active_ = false;
      }
      cancel_when_accepted_ = true;
      immediate_message = "Cancel deferred until the Action goal response arrives";
    } else if (command_state_ == CommandState::kCanceling) {
      immediate_message = "CANCELING: cancellation is already in progress";
    } else {
      if (move_pose_relative_sequence_active_) {
        move_pose_relative_sequence_active_ = false;
      }
      command_state_ = CommandState::kCanceling;
      cancel_when_accepted_ = true;
      command_type = command_type_;
      move_j_goal = active_move_j_goal_;
      move_pose_goal = active_move_pose_goal_;
    }
  }

  if (!immediate_message.empty()) {
    printOutput(immediate_message);
    return;
  }
  if (command_type == CommandType::kMoveJ && move_j_goal) {
    requestMoveJCancellation(move_j_goal);
  } else if (command_type == CommandType::kMovePose && move_pose_goal) {
    requestMovePoseCancellation(move_pose_goal);
  } else {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_state_ = CommandState::kActive;
      cancel_when_accepted_ = false;
    }
    printOutput("ERROR: active command has no matching goal handle");
  }
}

void ConsoleNode::handleStopCommand()
{
  bool restore_active_on_failure = false;
  bool sequence_canceled = false;
  CommandType stopped_type = CommandType::kNone;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    stopped_type = command_type_;
    if (command_state_ == CommandState::kWaiting &&
      move_pose_relative_sequence_active_)
    {
      move_pose_relative_sequence_active_ = false;
      if (move_pose_relative_sequence_timer_) {
        move_pose_relative_sequence_timer_->cancel();
        move_pose_relative_sequence_timer_.reset();
      }
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      sequence_canceled = true;
    } else if (command_state_ == CommandState::kSending) {
      if (move_pose_relative_sequence_active_) {
        move_pose_relative_sequence_active_ = false;
        sequence_canceled = true;
      }
      cancel_when_accepted_ = true;
    } else if (command_state_ == CommandState::kActive) {
      if (move_pose_relative_sequence_active_) {
        move_pose_relative_sequence_active_ = false;
        sequence_canceled = true;
      }
      command_state_ = CommandState::kCanceling;
      restore_active_on_failure = true;
    }
  }
  if (sequence_canceled) {
    printOutput("MovePosePTP sequence canceled by stop request");
  }

  const auto timeout = std::chrono::duration<double>(stop_service_timeout_sec_);
  auto restore_active = [this, restore_active_on_failure, stopped_type]()
    {
      if (!restore_active_on_failure) {
        return;
      }
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_state_ == CommandState::kCanceling && command_type_ == stopped_type) {
        command_state_ = CommandState::kActive;
      }
    };

  if (!stop_client_->wait_for_service(timeout)) {
    restore_active();
    printOutput("ERROR: /elfin3_motion/stop is unavailable");
    return;
  }

  try {
    const auto request = std::make_shared<SupervisorTrigger::Request>();
    auto future = stop_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      stop_client_->remove_pending_request(future);
      restore_active();
      printOutput("ERROR: /elfin3_motion/stop timed out");
      return;
    }

    const auto response = future.get();
    if (!response->success) {
      restore_active();
    }
    printOutput(
      std::string("Stop response: ") + (response->success ? "SUCCESS" : "FAILED") +
      " message='" + response->message + "'");
  } catch (const std::exception & exception) {
    restore_active();
    printOutput(std::string("ERROR: stop request failed: ") + exception.what());
  }
}

bool ConsoleNode::querySupervisorReadiness(std::string & reason)
{
  const auto timeout = std::chrono::duration<double>(supervisor_timeout_sec_);
  if (!supervisor_client_->wait_for_service(timeout)) {
    reason = "/elfin3_supervisor/is_ready is unavailable";
    return false;
  }

  try {
    const auto request = std::make_shared<SupervisorTrigger::Request>();
    auto future = supervisor_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      supervisor_client_->remove_pending_request(future);
      reason = "/elfin3_supervisor/is_ready timed out";
      return false;
    }

    const auto response = future.get();
    reason = response->message.empty() ?
      "supervisor returned an empty message" : response->message;
    return response->success;
  } catch (const std::exception & exception) {
    reason = std::string("supervisor readiness request failed: ") + exception.what();
    return false;
  }
}

void ConsoleNode::requestMoveJCancellation(
  const MoveJGoalHandle::SharedPtr & goal_handle)
{
  try {
    move_j_client_->async_cancel_goal(goal_handle);
    printOutput("MoveJ cancel request submitted: state=CANCELING");
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_state_ == CommandState::kCanceling &&
        command_type_ == CommandType::kMoveJ)
      {
        command_state_ = CommandState::kActive;
        cancel_when_accepted_ = false;
      }
    }
    printOutput(std::string("ERROR: failed to cancel MoveJ: ") + exception.what());
  }
}

void ConsoleNode::requestMovePoseCancellation(
  const MovePoseGoalHandle::SharedPtr & goal_handle)
{
  rclcpp_action::Client<MovePose>::SharedPtr action_client;
  std::string command_name;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    action_client = active_move_pose_client_;
    command_name = active_move_pose_name_;
  }

  try {
    if (!action_client) {
      throw std::runtime_error("active MovePose client is unavailable");
    }
    action_client->async_cancel_goal(goal_handle);
    printOutput(command_name + " cancel request submitted: state=CANCELING");
  } catch (const std::exception & exception) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_state_ == CommandState::kCanceling &&
        command_type_ == CommandType::kMovePose)
      {
        command_state_ = CommandState::kActive;
        cancel_when_accepted_ = false;
      }
    }
    printOutput(
      std::string("ERROR: failed to cancel ") + command_name + ": " + exception.what());
  }
}

void ConsoleNode::cancelActiveGoalForShutdown()
{
  if (shutdown_cleanup_started_.exchange(true)) {
    return;
  }

  MoveJGoalHandle::SharedPtr move_j_goal;
  MovePoseGoalHandle::SharedPtr move_pose_goal;
  CommandType command_type = CommandType::kNone;
  bool waiting_for_goal_response = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_state_ == CommandState::kWaiting &&
      move_pose_relative_sequence_active_)
    {
      move_pose_relative_sequence_active_ = false;
      if (move_pose_relative_sequence_timer_) {
        move_pose_relative_sequence_timer_->cancel();
        move_pose_relative_sequence_timer_.reset();
      }
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      return;
    }
    if (command_state_ == CommandState::kIdle ||
      command_state_ == CommandState::kCanceling)
    {
      return;
    }

    if (move_pose_relative_sequence_active_) {
      move_pose_relative_sequence_active_ = false;
      if (move_pose_relative_sequence_timer_) {
        move_pose_relative_sequence_timer_->cancel();
        move_pose_relative_sequence_timer_.reset();
      }
    }

    if (command_state_ == CommandState::kSending) {
      cancel_when_accepted_ = true;
      waiting_for_goal_response = true;
    } else {
      command_state_ = CommandState::kCanceling;
      cancel_when_accepted_ = true;
      command_type = command_type_;
      move_j_goal = active_move_j_goal_;
      move_pose_goal = active_move_pose_goal_;
    }
  }

  if (waiting_for_goal_response) {
    RCLCPP_WARN(
      get_logger(),
      "Shutdown started while an Action goal response was pending; "
      "marked it for cancellation if the response arrives in time");
    return;
  }

  RCLCPP_WARN(get_logger(), "Shutdown requested with an active console motion goal");
  if (command_type == CommandType::kMoveJ && move_j_goal) {
    requestMoveJCancellation(move_j_goal);
  } else if (command_type == CommandType::kMovePose && move_pose_goal) {
    requestMovePoseCancellation(move_pose_goal);
  } else {
    RCLCPP_ERROR(
      get_logger(), "Unable to issue shutdown cancellation: goal handle is unavailable");
  }
}

void ConsoleNode::handleMoveJGoalResponse(
  const MoveJGoalHandle::SharedPtr & goal_handle)
{
  bool cancel_immediately = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!goal_handle) {
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      cancel_when_accepted_ = false;
      active_move_j_goal_.reset();
    } else {
      active_move_j_goal_ = goal_handle;
      cancel_immediately = cancel_when_accepted_;
      command_state_ = cancel_immediately ?
        CommandState::kCanceling : CommandState::kActive;
    }
  }

  if (!goal_handle) {
    printOutput("MoveJ goal rejected by server");
  } else if (cancel_immediately) {
    printOutput("MoveJ goal accepted with deferred cancellation");
    requestMoveJCancellation(goal_handle);
  } else {
    printOutput("MoveJ goal accepted: state=ACTIVE");
  }
}

void ConsoleNode::handleMoveJFeedback(
  MoveJGoalHandle::SharedPtr,
  const std::shared_ptr<const MoveJ::Feedback> feedback)
{
  if (feedback) {
    printOutput("MoveJ feedback: " + feedback->stage);
  }
}

void ConsoleNode::handleMoveJResult(
  const MoveJGoalHandle::WrappedResult & wrapped_result)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    active_move_j_goal_.reset();
    active_move_pose_goal_.reset();
    command_state_ = CommandState::kIdle;
    command_type_ = CommandType::kNone;
    cancel_when_accepted_ = false;
  }

  std::ostringstream output;
  output << "MoveJ result: action=" << actionResultCodeText(wrapped_result.code);
  if (wrapped_result.result) {
    output << " result_code=" << static_cast<unsigned int>(
      wrapped_result.result->result_code) <<
      " moveit_error_code=" << wrapped_result.result->moveit_error_code <<
      " message='" << wrapped_result.result->message << '\'';
  } else {
    output << " result payload unavailable";
  }
  printOutput(output.str());
}

void ConsoleNode::handleMovePoseGoalResponse(
  const MovePoseGoalHandle::SharedPtr & goal_handle)
{
  bool cancel_immediately = false;
  bool sequence_aborted = false;
  std::size_t sequence_step = 0;
  std::string command_name;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_name = active_move_pose_name_;
    if (!goal_handle) {
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      cancel_when_accepted_ = false;
      active_move_pose_goal_.reset();
      active_move_pose_client_.reset();
      active_move_pose_name_ = "MovePose";
      if (move_pose_relative_sequence_active_) {
        sequence_step = move_pose_relative_sequence_step_;
        move_pose_relative_sequence_active_ = false;
        if (move_pose_relative_sequence_timer_) {
          move_pose_relative_sequence_timer_->cancel();
          move_pose_relative_sequence_timer_.reset();
        }
        sequence_aborted = true;
      }
    } else {
      active_move_pose_goal_ = goal_handle;
      cancel_immediately = cancel_when_accepted_;
      command_state_ = cancel_immediately ?
        CommandState::kCanceling : CommandState::kActive;
    }
  }

  if (!goal_handle) {
    printOutput(command_name + " goal rejected by server");
    if (sequence_aborted) {
      printOutput(
        "MovePosePTP sequence aborted: step " +
        std::to_string(sequence_step + 1) + " goal was rejected");
    }
  } else if (cancel_immediately) {
    printOutput(command_name + " goal accepted with deferred cancellation");
    requestMovePoseCancellation(goal_handle);
  } else {
    printOutput(command_name + " goal accepted: state=ACTIVE");
  }
}

void ConsoleNode::handleMovePoseFeedback(
  MovePoseGoalHandle::SharedPtr,
  const std::shared_ptr<const MovePose::Feedback> feedback)
{
  if (feedback) {
    std::string command_name;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_name = active_move_pose_name_;
    }
    printOutput(command_name + " feedback: " + feedback->stage);
  }
}

void ConsoleNode::handleMovePoseResult(
  const MovePoseGoalHandle::WrappedResult & wrapped_result)
{
  const bool succeeded =
    wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped_result.result &&
    wrapped_result.result->result_code == MovePose::Result::SUCCESS;
  std::string command_name;
  bool schedule_next_step = false;
  bool sequence_completed = false;
  bool sequence_aborted = false;
  std::size_t completed_step = 0;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_name = active_move_pose_name_;
    active_move_j_goal_.reset();
    active_move_pose_goal_.reset();
    command_type_ = CommandType::kNone;
    cancel_when_accepted_ = false;
    active_move_pose_client_.reset();
    active_move_pose_name_ = "MovePose";
    if (move_pose_relative_sequence_active_) {
      completed_step = move_pose_relative_sequence_step_;
      if (succeeded) {
        ++move_pose_relative_sequence_step_;
        if (move_pose_relative_sequence_step_ <
          kRelativeSequenceNegativeTranslationsCm.size())
        {
          command_state_ = CommandState::kWaiting;
          schedule_next_step = true;
        } else {
          move_pose_relative_sequence_active_ = false;
          command_state_ = CommandState::kIdle;
          sequence_completed = true;
        }
      } else {
        move_pose_relative_sequence_active_ = false;
        command_state_ = CommandState::kIdle;
        sequence_aborted = true;
      }
      if (!schedule_next_step && move_pose_relative_sequence_timer_) {
        move_pose_relative_sequence_timer_->cancel();
        move_pose_relative_sequence_timer_.reset();
      }
    } else {
      command_state_ = CommandState::kIdle;
    }
  }

  std::ostringstream output;
  output << command_name << " result: action=" << actionResultCodeText(wrapped_result.code);
  if (wrapped_result.result) {
    output << " result_code=" << static_cast<unsigned int>(
      wrapped_result.result->result_code) <<
      " moveit_error_code=" << wrapped_result.result->moveit_error_code <<
      " message='" << wrapped_result.result->message << '\'';
  } else {
    output << " result payload unavailable";
  }
  printOutput(output.str());
  if (schedule_next_step) {
    scheduleMovePoseRelativeSequenceStep();
  } else if (sequence_completed) {
    printOutput("MovePosePTP sequence completed successfully");
  } else if (sequence_aborted) {
    printOutput(
      "MovePosePTP sequence aborted at step " +
      std::to_string(completed_step + 1));
  }
}

/**
 * @brief 循环读取并处理控制台命令
 */
void ConsoleNode::inputLoop()
{
  printOutput("Elfin3 console ready; type 'help' for commands");

  pollfd input_descriptor{};
  input_descriptor.fd = STDIN_FILENO;
  input_descriptor.events = POLLIN;

  while (rclcpp::ok() && !input_stop_requested_.load()) {
    input_descriptor.revents = 0;
    const int poll_result =
      ::poll(&input_descriptor, 1, static_cast<int>(input_poll_timeout_ms_));

    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      printOutput(
        std::string("ERROR: stdin poll failed: ") + std::strerror(errno), false);
      prepareForShutdown();
      rclcpp::shutdown();
      return;
    }
    if (poll_result == 0) {
      continue;
    }

    if ((input_descriptor.revents & POLLIN) != 0) {
      std::string line;
      if (!std::getline(std::cin, line)) {
        prepareForShutdown();
        printOutput("stdin closed; exiting elfin3_console", false);
        rclcpp::shutdown();
        return;
      }
      handleCommand(line);
      continue;
    }

    if ((input_descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      prepareForShutdown();
      printOutput("stdin is unavailable; exiting elfin3_console", false);
      rclcpp::shutdown();
      return;
    }
  }
}

void ConsoleNode::printOutput(const std::string & text, bool show_prompt)
{
  if (!terminal_output_) {
    return;
  }

  std::lock_guard<std::mutex> lock(output_mutex_);
  std::cout << text << '\n';
  if (show_prompt && !input_stop_requested_.load() && rclcpp::ok()) {
    std::cout << "elfin3> " << std::flush;
  } else {
    std::cout << std::flush;
  }
}

void ConsoleNode::printPrompt()
{
  if (!terminal_output_ || input_stop_requested_.load() || !rclcpp::ok()) {
    return;
  }

  std::lock_guard<std::mutex> lock(output_mutex_);
  std::cout << "elfin3> " << std::flush;
}

}  // namespace elfin3_console
