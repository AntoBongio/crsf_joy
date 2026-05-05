// crsf_joy_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// CRSF constants
static constexpr uint8_t  CRSF_SYNC        = 0xC8;
static constexpr uint8_t  CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
static constexpr int      CRSF_NUM_CHANNELS = 16;
static constexpr double   CRSF_CENTER      = 992.0;
static constexpr double   CRSF_RANGE       = 819.0; // (1811-172)/2

// Decode 16 x 11-bit channels from 22-byte payload
static void decode_channels(const uint8_t* payload, uint16_t* ch)
{
  // Standard CRSF 11-bit channel unpacking
  ch[0]  = ((payload[0]       | payload[1]  << 8) & 0x07FF);
  ch[1]  = ((payload[1]  >> 3 | payload[2]  << 5) & 0x07FF);
  ch[2]  = ((payload[2]  >> 6 | payload[3]  << 2 | payload[4] << 10) & 0x07FF);
  ch[3]  = ((payload[4]  >> 1 | payload[5]  << 7) & 0x07FF);
  ch[4]  = ((payload[5]  >> 4 | payload[6]  << 4) & 0x07FF);
  ch[5]  = ((payload[6]  >> 7 | payload[7]  << 1 | payload[8] << 9)  & 0x07FF);
  ch[6]  = ((payload[8]  >> 2 | payload[9]  << 6) & 0x07FF);
  ch[7]  = ((payload[9]  >> 5 | payload[10] << 3) & 0x07FF);
  ch[8]  = ((payload[11]      | payload[12] << 8) & 0x07FF);
  ch[9]  = ((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
  ch[10] = ((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
  ch[11] = ((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
  ch[12] = ((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
  ch[13] = ((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9)  & 0x07FF);
  ch[14] = ((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
  ch[15] = ((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
}

class CrsfJoyNode : public rclcpp::Node
{
public:
  CrsfJoyNode() : Node("crsf_joy_node")
  {
    port_name_ = declare_parameter<std::string>("port", "/dev/xr4_crsf");
    // Threshold above which a switch axis counts as a button press
    switch_threshold_ = declare_parameter<double>("switch_threshold", 0.5);
    // Which channel indices to expose as Joy buttons[] instead of axes[]
    // Example: channels 4..15 → buttons, 0..3 → axes
    button_channels_ = declare_parameter<std::vector<int64_t>>(
      "button_channels", {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});

    pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS());

    open_serial();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&CrsfJoyNode::read_serial, this));
  }

private:
  void open_serial()
  {
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Cannot open %s", port_name_.c_str());
      throw std::runtime_error("serial open failed");
    }
    struct termios tty {};
    cfsetispeed(&tty, B460800);   // closest standard; actual 420000 via custom divisor
    cfsetospeed(&tty, B460800);
    // For exact 420000 baud, use custom_divisor via TIOCGSERIAL/TIOCSSERIAL
    // or rely on the CP2102/CH343 driver which typically supports 420000 natively.
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tcsetattr(fd_, TCSANOW, &tty);
  }

  void read_serial()
  {
    uint8_t byte;
    while (read(fd_, &byte, 1) == 1) {
      buf_[buf_len_++] = byte;
      if (buf_len_ >= 2 && buf_[0] != CRSF_SYNC) {
        // Resync: shift buffer
        memmove(buf_, buf_ + 1, --buf_len_);
        continue;
      }
      if (buf_len_ < 2) continue;
      uint8_t frame_len = buf_[1]; // payload length + type byte
      if (buf_len_ < static_cast<size_t>(frame_len + 2)) continue;

      if (buf_[2] == CRSF_FRAMETYPE_RC_CHANNELS) {
        uint16_t ch[CRSF_NUM_CHANNELS];
        decode_channels(&buf_[3], ch);
        publish_joy(ch);
      }
      // Consume frame
      size_t total = frame_len + 2;
      memmove(buf_, buf_ + total, buf_len_ - total);
      buf_len_ -= total;
    }
  }

  void publish_joy(const uint16_t* ch)
  {
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = now();

    // Decide which channels go to axes vs buttons
    std::set<int> btn_set(button_channels_.begin(), button_channels_.end());

    for (int i = 0; i < CRSF_NUM_CHANNELS; ++i) {
      double norm = (static_cast<double>(ch[i]) - CRSF_CENTER) / CRSF_RANGE;
      norm = std::clamp(norm, -1.0, 1.0);
      if (btn_set.count(i)) {
        msg.buttons.push_back(norm > switch_threshold_ ? 1 : 0);
      } else {
        msg.axes.push_back(static_cast<float>(norm));
      }
    }
    pub_->publish(msg);
  }

  int fd_ {-1};
  uint8_t buf_[64] {};
  size_t buf_len_ {0};
  std::string port_name_;
  double switch_threshold_;
  std::vector<int64_t> button_channels_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrsfJoyNode>());
  rclcpp::shutdown();
}
