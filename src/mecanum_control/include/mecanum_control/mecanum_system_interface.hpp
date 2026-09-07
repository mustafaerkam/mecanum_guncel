#ifndef MECANUM_CONTROL__MECANUM_SYSTEM_INTERFACE_HPP_
#define MECANUM_CONTROL__MECANUM_SYSTEM_INTERFACE_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace mecanum_control
{

// Bu arayuz KASITLI olarak protokolden bagimsizdir: SystemInterface icine
// mecanum kinematigi, teker PID'si veya Nav2 mantigi konmaz. Kinematik
// mecanum_drive_controller'da, teker PID'i ESP32 firmware'indedir.
//
// Tasima katmani: ESP32 uzerinde micro-ROS calisir ve USB seri uzerinden
// micro_ros_agent ile ROS 2 grafina baglanir. Yani ozel bir byte protokolu
// YOKTUR; ESP normal bir ROS node'udur ve asagidaki iki topic ile konusur.
class WheelTransport
{
public:
  struct WheelFeedback
  {
    double position = 0.0;  // rad, kumulatif, joint eksenine gore isaretli
    double velocity = 0.0;  // rad/s
  };

  virtual ~WheelTransport() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  // Diziler DAIMA {FL, FR, RL, RR} sirasindadir.
  virtual bool read_feedback(
    std::array<WheelFeedback, 4> & out, const rclcpp::Duration & period) = 0;
  virtual bool write_velocity_command(const std::array<double, 4> & wheel_velocities_rad_s) = 0;
  virtual bool is_connected() const = 0;
};

// Donanimsiz test icin: ros2_control lifecycle'ini ve komut zincirini
// dogrulamak icin kullanilir. Gercek fiziksel deger uretmez.
class MockTransport : public WheelTransport
{
public:
  bool open() override;
  void close() override;
  bool read_feedback(
    std::array<WheelFeedback, 4> & out, const rclcpp::Duration & period) override;
  bool write_velocity_command(const std::array<double, 4> & wheel_velocities_rad_s) override;
  bool is_connected() const override {return connected_;}

private:
  bool connected_ = false;
  std::array<double, 4> commanded_velocity_{};
  std::array<double, 4> integrated_position_{};
};

// micro-ROS tasima katmani.
//
//   Pi  -> ESP : <command_topic>  std_msgs/Float32MultiArray
//                data[4] = {FL, FR, RL, RR} hedef teker hizi [rad/s]
//                HER kontrol cevriminde kosulsuz yayinlanir; boylece ESP
//                firmware'inin watchdog'u sabit hizda surerken tetiklenmez.
//
//   ESP -> Pi  : <state_topic>    std_msgs/Float32MultiArray
//                data[8] = {pos_FL, pos_FR, pos_RL, pos_RR,
//                           vel_FL, vel_FR, vel_RL, vel_RR}
//                pozisyon kumulatif rad (sarmalanmamis), hiz rad/s
//
// MultiArrayLayout KULLANILMAZ (bos birakilir): micro-ROS tarafinda dinamik
// bellek ayrimini azaltir. Eleman sirasi sozlesmenin kendisidir.
class MicroRosTransport : public WheelTransport
{
public:
  struct Config
  {
    std::string command_topic = "wheel/commands";
    std::string state_topic = "wheel/states";
    bool best_effort_qos = true;   // 50 Hz periyodik setpoint icin dogru secim
    double state_timeout = 0.2;    // s; bu sureden eski feedback BAYAT sayilir
    double activation_timeout = 5.0;  // s; ilk feedback icin on_activate beklemesi
    bool open_loop = false;        // true: feedback yoksa komutu geri yansit
  };

  explicit MicroRosTransport(const Config & config);
  ~MicroRosTransport() override;

  bool open() override;
  void close() override;
  bool read_feedback(
    std::array<WheelFeedback, 4> & out, const rclcpp::Duration & period) override;
  bool write_velocity_command(const std::array<double, 4> & wheel_velocities_rad_s) override;
  bool is_connected() const override {return connected_;}

private:
  void on_state(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  Config config_;
  bool connected_ = false;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_subscription_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::thread spin_thread_;

  mutable std::mutex state_mutex_;
  std::array<WheelFeedback, 4> last_state_{};
  rclcpp::Time last_state_time_{0, 0, RCL_STEADY_TIME};
  bool state_received_ = false;

  // open_loop modunda kullanilir
  std::array<double, 4> commanded_velocity_{};
  std::array<double, 4> integrated_position_{};

  std_msgs::msg::Float32MultiArray command_msg_;
};

class MecanumSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kNumWheels = 4;

  // ros2_control joint'leri ISIMLE eslestirir, bu yuzden info_.joints sirasi
  // garanti degildir. wheel_slot_[k] = k'inci teker yuvasinin (FL,FR,RL,RR)
  // info_.joints icindeki indeksi. Bu esleme olmadan xacro'daki joint sirasi
  // degistiginde teker komutlari SESSIZCE yer degistirir.
  std::array<std::size_t, kNumWheels> wheel_slot_{};

  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_position_states_;

  std::unique_ptr<WheelTransport> transport_;
};

}  // namespace mecanum_control

#endif  // MECANUM_CONTROL__MECANUM_SYSTEM_INTERFACE_HPP_
