# ydlidar_x2_parser
YDLiDAR製X2のスキャンデータを、M5ATOM Liteを使用して最終的に/scanトピックにするためのパッケージ
## 概要
M5ATOM LiteとX2を接続して使用する．
X2から送信されてきたパケットを，std_msgs/msg/String型の/lidar_rawトピックとして，PCへ送信する．
PC側では，M5ATOM Liteから送信されてきた/lidar_rawトピックを解釈し，sensor_msgs/msg/LaserScan型の/scanトピックへ変換する．
