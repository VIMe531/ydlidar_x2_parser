#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

class YDLidarX2Parser : public rclcpp::Node
{
public:
  YDLidarX2Parser()
  : Node("ydlidar_x2_parser")
  {
    // パラメータ宣言
    frame_id_  = this->declare_parameter<std::string>("frame_id", "laser_frame");
    range_min_ = this->declare_parameter<double>("range_min", 0.10);
    range_max_ = this->declare_parameter<double>("range_max", 8.00);
    scan_time_ = this->declare_parameter<double>("scan_time", 1.0 / 7.0);
    num_points_ = this->declare_parameter<int>("num_points", 360);

    ranges_.resize(num_points_, std::numeric_limits<float>::quiet_NaN());
    valid_.resize(num_points_, false);

    angle_sum_deg_ = 0.0;
    first_packet_in_rev_ = true;

    using std::placeholders::_1;
    raw_sub_ = this->create_subscription<std_msgs::msg::String>(
      "lidar_raw", 10,
      std::bind(&YDLidarX2Parser::rawCallback, this, _1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    RCLCPP_INFO(this->get_logger(), "YDLidar X2 parser node started.");
  }

private:
  // /lidar_raw コールバック
  void rawCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::vector<uint8_t> bytes;
    if (!parseHexString(msg->data, bytes)) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse hex string.");
      return;
    }

    if (bytes.size() < 10) {
      return;
    }

    int start_idx = findHeader(bytes);
    if (start_idx < 0) {
      return;
    }

    parsePacket(bytes, start_idx);
  }

  // "AA 55 00 28 ..." → [0xAA, 0x55, ...]
  bool parseHexString(const std::string &s, std::vector<uint8_t> &out)
  {
    out.clear();

    std::istringstream iss(s);
    std::string token;

    while (iss >> token) {
      if (token.empty()) {
        continue;
      }
      // "data:" のようなプレフィクスをスキップ
      if (token.back() == ':') {
        continue;
      }

      // "0xAA" 形式への対応
      if (token.rfind("0x", 0) == 0 || token.rfind("0X", 0) == 0) {
        token = token.substr(2);
      }

      // 大文字小文字どちらでもOKにしたいので変換
      std::transform(token.begin(), token.end(), token.begin(), ::toupper);

      // 2桁16進数想定だが、長さチェックは一応しない
      try {
        uint8_t val = static_cast<uint8_t>(std::stoul(token, nullptr, 16));
        out.push_back(val);
      } catch (...) {
        return false;
      }
    }

    return !out.empty();
  }

  // "AA 55" の位置を探す
  int findHeader(const std::vector<uint8_t> &data)
  {
    if (data.size() < 2) {
      return -1;
    }
    for (size_t i = 0; i + 1 < data.size(); ++i) {
      if (data[i] == 0xAA && data[i + 1] == 0x55) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  // 角度のデコード
  float decodeAngle(uint8_t lo, uint8_t hi)
  {
    uint16_t raw = (static_cast<uint16_t>(hi) << 8) | lo;
    raw = (raw >> 1) & 0x7FFF;
    return static_cast<float>(raw) / 64.0f;  // deg
  }

  // 1 パケット分の処理
  void parsePacket(const std::vector<uint8_t> &data, int start_idx)
  {
    if (data.size() < static_cast<size_t>(start_idx + 10)) {
      return;
    }

    uint8_t ct    = data[start_idx + 2];
    (void)ct;  // 今回は使わないが、将来拡張用に残す
    uint8_t lsn   = data[start_idx + 3];
    uint8_t fsa_l = data[start_idx + 4];
    uint8_t fsa_h = data[start_idx + 5];
    uint8_t lsa_l = data[start_idx + 6];
    uint8_t lsa_h = data[start_idx + 7];
    // uint8_t cs_l = data[start_idx + 8];
    // uint8_t cs_h = data[start_idx + 9];

    int sample_start = start_idx + 10;
    int need_bytes = static_cast<int>(lsn) * 2;

    if (lsn == 0) {
      return;
    }
    if (data.size() < static_cast<size_t>(sample_start + need_bytes)) {
      return;
    }

    float start_angle_deg = decodeAngle(fsa_l, fsa_h);
    float end_angle_deg   = decodeAngle(lsa_l, lsa_h);

    if (end_angle_deg < start_angle_deg) {
      end_angle_deg += 360.0f;
    }
    float delta_angle_deg = end_angle_deg - start_angle_deg;

    // 1 回転判定用の角度合計
    if (first_packet_in_rev_) {
      angle_sum_deg_ = delta_angle_deg;
      first_packet_in_rev_ = false;
    } else {
      angle_sum_deg_ += delta_angle_deg;
    }

    // 各サンプルを 0〜359 のバケツに格納
    for (int i = 0; i < static_cast<int>(lsn); ++i) {
      int offset = sample_start + 2 * i;
      uint8_t dist_l = data[offset];
      uint8_t dist_h = data[offset + 1];
      uint16_t raw = (static_cast<uint16_t>(dist_h) << 8) | dist_l;

      uint16_t raw_dist = raw & 0x3FFF;
      float dist_m;
      if (raw_dist == 0) {
        dist_m = std::numeric_limits<float>::quiet_NaN();
      } else {
        dist_m = static_cast<float>(raw_dist) / 4000.0f;  // m
      }

      float t = 0.0f;
      if (lsn > 1) {
        t = static_cast<float>(i) / static_cast<float>(lsn - 1);
      }
      float angle_deg = start_angle_deg + delta_angle_deg * t;

      // 0〜360 に正規化
      while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
      }
      while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
      }

      int idx = static_cast<int>(std::floor(angle_deg + 0.5f));  // 最近傍
      if (idx < 0 || idx >= num_points_) {
        continue;
      }

      if (!std::isnan(dist_m) &&
          dist_m >= static_cast<float>(range_min_) &&
          dist_m <= static_cast<float>(range_max_)) {
        ranges_[idx] = dist_m;
        valid_[idx] = true;
      } else {
        ranges_[idx] = std::numeric_limits<float>::quiet_NaN();
        valid_[idx] = false;
      }
    }

    // 1 回転分たまったら publish
    if (angle_sum_deg_ >= 360.0f) {
      publishScan();
      resetScanBuffer();
    }
  }

  void publishScan()
  {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->now();
    scan.header.frame_id = frame_id_;

    scan.angle_min = 0.0f;
    scan.angle_max = 2.0f * static_cast<float>(M_PI);
    scan.angle_increment =
      (scan.angle_max - scan.angle_min) / static_cast<float>(num_points_);
    scan.time_increment = 0.0f;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);

    scan.ranges.resize(num_points_);
    scan.intensities.resize(num_points_);
    for (int i = 0; i < num_points_; ++i) {
      if (valid_[i]) {
        scan.ranges[i] = ranges_[i];
      } else {
        scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
      scan.intensities[i] = 0.0f;
    }

    scan_pub_->publish(scan);
  }

  void resetScanBuffer()
  {
    std::fill(ranges_.begin(), ranges_.end(),
              std::numeric_limits<float>::quiet_NaN());
    std::fill(valid_.begin(), valid_.end(), false);
    angle_sum_deg_ = 0.0;
    first_packet_in_rev_ = true;
  }

  // メンバ変数
  std::string frame_id_;
  double range_min_;
  double range_max_;
  double scan_time_;
  int num_points_;

  std::vector<float> ranges_;
  std::vector<bool>  valid_;

  double angle_sum_deg_;
  bool   first_packet_in_rev_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YDLidarX2Parser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

