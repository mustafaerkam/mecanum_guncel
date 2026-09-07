#include "mecanum_control/mecanum_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/logging.hpp"

namespace mecanum_control
{

namespace
{
rclcpp::Logger logger() {return rclcpp::get_logger("mecanum_system_interface");}

// {FL, FR, RL, RR} yuva sirasi. Bu sira TUM tasima katmanlari icin sabittir.
const std::array<std::string, 4> kDefaultWheelJoints{
  "wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};

std::string param_or(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end() && !it->second.empty()) ? it->second : fallback;
}

std::vector<std::string> split_csv(const std::string & value)
{
  std::vector<std::string> out;
  std::string token;
  for (const char c : value) {
    if (c == ',') {
      if (!token.empty()) {out.push_back(token);}
      token.clear();
    } else if (c != ' ') {
      token.push_back(c);
    }
  }
  if (!token.empty()) {out.push_back(token);}
  return out;
}
}  // namespace

// ── MockTransport ────────────────────────────────────────────────
bool MockTransport::open()
{
  connected_ = true;
  commanded_velocity_.fill(0.0);
  integrated_position_.fill(0.0);
  return true;
}

void MockTransport::close() {connected_ = false;}

bool MockTransport::read_feedback(
  std::array<WheelFeedback, 4> & out, const rclcpp::Duration & period)
{
  if (!connected_) {return false;}
  const double dt = period.seconds();
  for (std::size_t i = 0; i < 4; ++i) {
    integrated_position_[i] += commanded_velocity_[i] * dt;
    out[i].velocity = commanded_velocity_[i];
    out[i].position = integrated_position_[i];
  }
  return true;
}

bool MockTransport::write_velocity_command(const std::array<double, 4> & wheel_velocities_rad_s)
{
  if (!connected_) {return false;}
  commanded_velocity_ = wheel_velocities_rad_s;
  return true;
}

// ── MicroRosTransport ────────────────────────────────────────────
MicroRosTransport::MicroRosTransport(const Config & config)
: config_(config)
{
  command_msg_.data.resize(4, 0.0F);
}

MicroRosTransport::~MicroRosTransport() {close();}

bool MicroRosTransport::open()
{
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(logger(), "rclcpp baslatilmamis; micro-ROS tasima katmani acilamaz.");
    return false;
  }

  // Kendi node'u: Humble'da SystemInterface bir node handle vermez.
  node_ = std::make_shared<rclcpp::Node>("mecanum_wheel_bridge");

  // 50 Hz periyodik setpoint akisi icin best_effort dogru secimdir: kayip bir
  // komut 20 ms sonra zaten yenisiyle gecersizlesir, reliable ise micro-ROS
  // XRCE akisinda yeniden gonderim ve tampon baskisi yaratir. Tek atimlik veri
  // olmadigi icin guvenilirlikten kazanc yoktur.
  const auto qos = config_.best_effort_qos
    ? rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
    : rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  command_publisher_ =
    node_->create_publisher<std_msgs::msg::Float32MultiArray>(config_.command_topic, qos);
  state_subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    config_.state_topic, qos,
    std::bind(&MicroRosTransport::on_state, this, std::placeholders::_1));

  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_received_ = false;
    last_state_ = {};
  }
  commanded_velocity_.fill(0.0);
  integrated_position_.fill(0.0);
  connected_ = true;

  RCLCPP_INFO(
    logger(), "micro-ROS tasima katmani acildi: komut='%s', durum='%s', QoS=%s",
    config_.command_topic.c_str(), config_.state_topic.c_str(),
    config_.best_effort_qos ? "best_effort" : "reliable");

  if (config_.open_loop) {
    RCLCPP_WARN(
      logger(),
      "feedback_mode=open_loop: teker geri beslemesi YOK sayilacak ve komut edilen hiz "
      "geri yansitilacak. Uretilen odometri GERCEK DEGILDIR, yalniz firmware hazir "
      "olmadan komut zincirini surmek icindir.");
    return true;
  }

  // Ilk feedback'i bekle: micro_ros_agent veya ESP hazir degilse burada acik
  // bir hata vermek, sessizce bayat veriyle surmekten iyidir.
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration<double>(config_.activation_timeout);
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_received_) {
        RCLCPP_INFO(logger(), "ESP32 teker durumu alindi, aktivasyon tamam.");
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  RCLCPP_ERROR(
    logger(),
    "'%s' uzerinde %.1f s icinde ESP32 teker durumu gelmedi. micro_ros_agent calisiyor mu, "
    "ESP32 bagli mi? Firmware olmadan surmek icin feedback_mode=open_loop kullanin.",
    config_.state_topic.c_str(), config_.activation_timeout);
  close();
  return false;
}

void MicroRosTransport::close()
{
  connected_ = false;
  if (executor_) {executor_->cancel();}
  if (spin_thread_.joinable()) {spin_thread_.join();}
  command_publisher_.reset();
  state_subscription_.reset();
  executor_.reset();
  node_.reset();
}

void MicroRosTransport::on_state(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 8) {
    RCLCPP_ERROR_THROTTLE(
      logger(), *node_->get_clock(), 5000,
      "'%s' uzerinde 8 eleman bekleniyor {4x pozisyon, 4x hiz}, %zu geldi. Mesaj yok sayildi.",
      config_.state_topic.c_str(), msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t i = 0; i < 4; ++i) {
    last_state_[i].position = static_cast<double>(msg->data[i]);
    last_state_[i].velocity = static_cast<double>(msg->data[i + 4]);
  }
  last_state_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  state_received_ = true;
}

bool MicroRosTransport::read_feedback(
  std::array<WheelFeedback, 4> & out, const rclcpp::Duration & period)
{
  if (!connected_) {return false;}

  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_received_) {
      const double age =
        (rclcpp::Clock(RCL_STEADY_TIME).now() - last_state_time_).seconds();
      if (age <= config_.state_timeout) {
        out = last_state_;
        fresh = true;
      }
    }
  }
  if (fresh) {return true;}

  if (!config_.open_loop) {
    RCLCPP_ERROR_THROTTLE(
      logger(), *node_->get_clock(), 2000,
      "ESP32 teker durumu %.2f s'den eski veya hic gelmedi ('%s').",
      config_.state_timeout, config_.state_topic.c_str());
    return false;
  }

  RCLCPP_WARN_THROTTLE(
    logger(), *node_->get_clock(), 5000,
    "open_loop: geri besleme yok, komut edilen hiz yansitiliyor. Odometri gercek degildir.");
  const double dt = period.seconds();
  for (std::size_t i = 0; i < 4; ++i) {
    integrated_position_[i] += commanded_velocity_[i] * dt;
    out[i].velocity = commanded_velocity_[i];
    out[i].position = integrated_position_[i];
  }
  return true;
}

bool MicroRosTransport::write_velocity_command(
  const std::array<double, 4> & wheel_velocities_rad_s)
{
  if (!connected_ || !command_publisher_) {return false;}
  commanded_velocity_ = wheel_velocities_rad_s;
  for (std::size_t i = 0; i < 4; ++i) {
    command_msg_.data[i] = static_cast<float>(wheel_velocities_rad_s[i]);
  }
  // KOSULSUZ yayin: degisim esigine bagli yayin, sabit hizda surerken topic'i
  // susturur ve ESP watchdog'unu tetikler.
  command_publisher_->publish(command_msg_);
  return true;
}

// ── MecanumSystemInterface ───────────────────────────────────────
hardware_interface::CallbackReturn MecanumSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumWheels) {
    RCLCPP_ERROR(
      logger(), "4 teker joint bekleniyor, %zu bulundu.", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // {FL, FR, RL, RR} yuvalarini joint isimleriyle esle. Indeks varsayimi YOK.
  auto wheel_joints = split_csv(
    param_or(
      info_, "wheel_joints",
      kDefaultWheelJoints[0] + "," + kDefaultWheelJoints[1] + "," +
      kDefaultWheelJoints[2] + "," + kDefaultWheelJoints[3]));
  if (wheel_joints.size() != kNumWheels) {
    RCLCPP_ERROR(
      logger(), "'wheel_joints' 4 isim icermeli (FL,FR,RL,RR sirasiyla), %zu bulundu.",
      wheel_joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t slot = 0; slot < kNumWheels; ++slot) {
    const auto it = std::find_if(
      info_.joints.begin(), info_.joints.end(),
      [&](const auto & joint) {return joint.name == wheel_joints[slot];});
    if (it == info_.joints.end()) {
      RCLCPP_ERROR(
        logger(), "'wheel_joints' icindeki '%s' URDF ros2_control joint'leri arasinda yok.",
        wheel_joints[slot].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    wheel_slot_[slot] = static_cast<std::size_t>(std::distance(info_.joints.begin(), it));
  }

  hw_velocity_commands_.assign(kNumWheels, 0.0);
  hw_velocity_states_.assign(kNumWheels, 0.0);
  hw_position_states_.assign(kNumWheels, 0.0);

  const std::string transport_type = param_or(info_, "transport", "mock");
  if (transport_type == "micro_ros") {
    MicroRosTransport::Config config;
    config.command_topic = param_or(info_, "command_topic", config.command_topic);
    config.state_topic = param_or(info_, "state_topic", config.state_topic);
    config.best_effort_qos = param_or(info_, "qos_reliability", "best_effort") != "reliable";
    config.state_timeout = std::stod(param_or(info_, "state_timeout", "0.2"));
    config.activation_timeout = std::stod(param_or(info_, "activation_timeout", "5.0"));
    config.open_loop = param_or(info_, "feedback_mode", "required") == "open_loop";
    transport_ = std::make_unique<MicroRosTransport>(config);
  } else if (transport_type == "mock") {
    transport_ = std::make_unique<MockTransport>();
    RCLCPP_WARN(
      logger(),
      "MecanumSystemInterface 'mock' tasima ile calisiyor - bu GERCEK ROBOT DEGIL, "
      "yalnizca ros2_control arayuz/lifecycle testi icindir.");
  } else {
    RCLCPP_ERROR(
      logger(), "Bilinmeyen transport='%s'. Gecerli: 'mock' | 'micro_ros'.",
      transport_type.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MecanumSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MecanumSystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
  if (!transport_->open()) {
    RCLCPP_ERROR(logger(), "Tasima katmani acilamadi.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Deaktivasyonda son is: sifir hiz gonder. ESP watchdog'u da bagimsiz olarak
  // durdurur, bu yalnizca ilk savunma hattidir.
  std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
  transport_->write_velocity_command({0.0, 0.0, 0.0, 0.0});
  transport_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::array<WheelTransport::WheelFeedback, kNumWheels> feedback{};
  if (!transport_->read_feedback(feedback, period)) {
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t slot = 0; slot < kNumWheels; ++slot) {
    const std::size_t joint = wheel_slot_[slot];
    hw_position_states_[joint] = feedback[slot].position;
    hw_velocity_states_[joint] = feedback[slot].velocity;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::array<double, kNumWheels> command{};
  for (std::size_t slot = 0; slot < kNumWheels; ++slot) {
    command[slot] = hw_velocity_commands_[wheel_slot_[slot]];
  }
  if (!transport_->write_velocity_command(command)) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mecanum_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanum_control::MecanumSystemInterface, hardware_interface::SystemInterface)
