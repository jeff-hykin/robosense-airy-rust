#define ENABLE_IMU_DATA_PARSE
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/driver/decoder/decoder_RSAIRY.hpp>
#include <cstdio>
#include <vector>
using namespace robosense::lidar;
typedef PointCloudT<PointXYZIRTF> Cloud;
int main(int argc, char** argv) {
  // argv: dump file, msop port, difop port, imu port, dense(0/1)
  FILE* f = fopen(argv[1], "rb");
  int msop = atoi(argv[2]), difop = atoi(argv[3]), imu = atoi(argv[4]);
  RSDecoderParam p;
  p.min_distance = 0.2; p.max_distance = 60; p.use_lidar_clock = true; p.dense_points = atoi(argv[5]); p.ts_first_point = true;
  DecoderRSAIRY<Cloud> d(p);
  d.point_cloud_ = std::make_shared<Cloud>();
  d.imuDataPtr_ = std::make_shared<ImuData>();
  uint32_t seq = 0;
  d.regCallback([](const Error& e){ }, [&](uint16_t h, double ts){
      if (d.point_cloud_->points.size() > 0) {
        printf("S %u %.6f %zu\n", seq++, ts, d.point_cloud_->points.size());
        for (auto& pt : d.point_cloud_->points) printf("%.6f %.6f %.6f %u %u %.6f %u\n", pt.x, pt.y, pt.z, pt.intensity, pt.ring, pt.timestamp, pt.feature);
        d.point_cloud_ = std::make_shared<Cloud>();
      }
  });
  d.regImuCallback([&](){ auto& m = *d.imuDataPtr_; printf("I %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", m.timestamp, m.linear_acceleration_x, m.linear_acceleration_y, m.linear_acceleration_z, m.angular_velocity_x, m.angular_velocity_y, m.angular_velocity_z); m.state = false; });
  uint16_t hdr[2]; std::vector<uint8_t> buf(65536);
  while (fread(hdr, 2, 2, f) == 2) {
    fread(buf.data(), 1, hdr[1], f);
    if (hdr[0] == difop) d.processDifopPkt(buf.data(), hdr[1]);
    else if (hdr[0] == msop) d.processMsopPkt(buf.data(), hdr[1]);
    else if (hdr[0] == imu) d.processImuPkt(buf.data(), hdr[1]);
  }
}
