#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

//==================================================
// ユーザ設定
//==================================================
#define WIFI_SSID "J90K5GPF"
#define WIFI_PASSWORD "35tansumachi2ss"
#define AGENT_IP "192.168.235.84"
#define AGENT_PORT 8888

//==================================================
// YDLiDAR X2 接続設定 (M5 Atom Lite 前提)
//==================================================
#define LIDAR_SERIAL Serial1
#define LIDAR_RX_PIN 22  // X2 の TX をここへ
#define LIDAR_BAUD 115200

#define LIDAR_PWM_PIN 25  // モータ用 PWM ピン（不要なら未接続で可）

//==================================================
// LED 設定
//==================================================
#define LED_PIN 27
#define NUM_PIX 1
#define PIX_R 0
#define PIX_G 1
#define PIX_B 2
#define PIX_Y 3
#define PIX_M 4
#define PIX_C 5
#define PIX_W 6
#define PIX_OFF 7

Adafruit_NeoPixel pixel(NUM_PIX, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t pix_colors[] = {
  pixel.Color(255, 0, 0),      // 赤
  pixel.Color(0, 255, 0),      // 緑
  pixel.Color(0, 0, 255),      // 青
  pixel.Color(255, 255, 0),    // 黄
  pixel.Color(255, 0, 255),    // マゼンタ
  pixel.Color(0, 255, 255),    // シアン
  pixel.Color(255, 255, 255),  // 白
  pixel.Color(0, 0, 0)         // 消灯
};

//==================================================
// micro-ROS グローバル
//==================================================
rcl_node_t node;
rcl_publisher_t packet_pub;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__String packet_msg;

//==================================================
// LiDAR パケットバッファ（シンプル）
//==================================================

// 1パケットの最大長（ヘッダ + データ）
static const size_t MAX_PACKET_BYTES = 200;

static uint8_t packet_buf[MAX_PACKET_BYTES];
static size_t packet_len = 0;

static uint8_t prev_byte = 0;
static bool have_prev = false;

//==================================================
// LEDの色を設定
//==================================================
void setLEDColor(uint8_t color_index) {
  if (color_index >= sizeof(pix_colors) / sizeof(pix_colors[0])) {
    return;
  }
  pixel.setPixelColor(0, pix_colors[color_index]);
  pixel.show();
}

//==================================================
// エラー時の待機
//==================================================
void error_loop(const char* msg) {
  Serial.println(msg);
  setLEDColor(PIX_R);
  while (1) {
    delay(1000);
  }
}

//==================================================
// LiDAR モータ用 PWM
//==================================================
void configure_lidar_pwm() {
  ledcSetup(0, 10000, 8);  // ch 0, 10kHz, 8bit
  ledcAttachPin(LIDAR_PWM_PIN, 0);
  ledcWrite(0, 255);  // フルスピード
}

//==================================================
// LiDAR スキャン開始（X2 は通電で回るのでダミー）
//==================================================
void send_start_scan_command() {
  // 必要なら制御コマンドをここで送る
}

//==================================================
// パケット1個分を 16進文字列に変換して publish
//==================================================
void publish_packet_hex(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return;
  }
  if (packet_msg.data.data == nullptr || packet_msg.data.capacity == 0) {
    return;
  }

  size_t cap = packet_msg.data.capacity;
  size_t pos = 0;

  // 1バイトあたり「2文字+スペース」で最大 3*len + 1
  for (size_t i = 0; i < len; ++i) {
    if (pos + 3 >= cap) {
      break;
    }

    uint8_t v = buf[i];
    uint8_t hi = (v >> 4) & 0x0F;
    uint8_t lo = v & 0x0F;

    char hi_c = (hi < 10) ? ('0' + hi) : ('A' + (hi - 10));
    char lo_c = (lo < 10) ? ('0' + lo) : ('A' + (lo - 10));

    packet_msg.data.data[pos++] = hi_c;
    packet_msg.data.data[pos++] = lo_c;
    packet_msg.data.data[pos++] = ' ';
  }

  // 末尾スペース削除（あっても困らないが、見た目のため）
  if (pos > 0 && packet_msg.data.data[pos - 1] == ' ') {
    pos -= 1;
  }

  // 終端
  if (pos >= cap) {
    pos = cap - 1;
  }
  packet_msg.data.data[pos] = '\0';
  packet_msg.data.size = pos;

  rcl_ret_t ret = rcl_publish(&packet_pub, &packet_msg, nullptr);
  if (ret != RCL_RET_OK) {
    Serial.print("[WARN] rcl_publish error: ");
    Serial.println((int)ret);
  } else {
    setLEDColor(PIX_B);  // 送信できたら青に
  }
}

//==================================================
// setup
//==================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("setup start");

  // M5 初期化
  auto cfg = M5.config();
  M5.begin(cfg);

  // NeoPixel 初期化
  pixel.begin();
  pixel.setBrightness(20);
  setLEDColor(PIX_OFF);

  // LiDAR PWM
  pinMode(LIDAR_PWM_PIN, OUTPUT);
  configure_lidar_pwm();

  // LiDAR UART (RX のみ)
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);

  Serial.println("LIDAR serial begun");

  // micro-ROS WiFi トランスポート設定
  Serial.println("set_microros_wifi_transports...");
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // support 初期化
  Serial.println("rclc_support_init...");
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    error_loop("rclc_support_init error");
  }

  // ノード作成
  Serial.println("rclc_node_init_default...");
  rc = rclc_node_init_default(
    &node,
    "ydlidar_packet_node",
    "",
    &support);
  if (rc != RCL_RET_OK) {
    error_loop("rclc_node_init_default error");
  }

  // Publisher 作成（std_msgs/String, トピック名 "lidar_raw"）
  Serial.println("rclc_publisher_init_default...");
  rc = rclc_publisher_init_default(
    &packet_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "lidar_raw");
  if (rc != RCL_RET_OK) {
    error_loop("rclc_publisher_init_default error");
  }

  // String メッセージのバッファ確保 (3*MAX_PACKET_BYTES+1)
  size_t max_str_len = 3 * MAX_PACKET_BYTES + 1;
  Serial.print("malloc for String, size=");
  Serial.println(max_str_len);

  // 念のためゼロクリア
  memset(&packet_msg, 0, sizeof(packet_msg));

  packet_msg.data.data = (char*)malloc(max_str_len);
  if (packet_msg.data.data == nullptr) {
    error_loop("malloc failed");
  }
  packet_msg.data.size = 0;
  packet_msg.data.capacity = max_str_len;
  packet_msg.data.data[0] = '\0';

  // LiDAR スタート（X2 は通電で回る）
  send_start_scan_command();

  // パケットバッファ初期化
  packet_len = 0;
  have_prev = false;

  setLEDColor(PIX_G);  // 初期化成功 → 緑

  Serial.println("setup finished");
}

//==================================================
// loop: LiDAR パケットを組み立てて、その場で publish
//==================================================
void loop() {
  // LiDAR からの受信処理
  while (LIDAR_SERIAL.available()) {
    uint8_t b = (uint8_t)LIDAR_SERIAL.read();

    // 「直前が 0xAA で、今回が 0x55」なら新パケットヘッダ
    if (have_prev && prev_byte == 0xAA && b == 0x55) {
      // すでに packet_len > 0 なら、それを1パケットとして publish
      if (packet_len > 0) {
        publish_packet_hex(packet_buf, packet_len);
      }

      // 新しいパケット開始
      packet_buf[0] = 0xAA;
      packet_buf[1] = 0x55;
      packet_len = 2;
    } else {
      // パケットバッファに追加（オーバーフローしたらリセット）
      if (packet_len < MAX_PACKET_BYTES) {
        packet_buf[packet_len++] = b;
      } else {
        // 取りすぎたら一旦破棄してヘッダ待ちに戻る
        packet_len = 0;
      }
    }

    prev_byte = b;
    have_prev = true;
  }

  // micro-ROS 内部処理のため少し待つ
  delay(1);
  M5.update();
}
