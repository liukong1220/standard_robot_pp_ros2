#include <cmath>

#include <gtest/gtest.h>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "standard_robot_pp_ros2/packet_typedef_addon.hpp"

namespace standard_robot_pp_ros2
{

TEST(GimbalProtocol, WireSizesAndCrc)
{
  EXPECT_EQ(sizeof(ReceiveGimbalStateData), 49U);
  EXPECT_EQ(sizeof(SendGimbalCmdData), 35U);

  SendGimbalCmdData packet{};
  packet.frame_header.sof = SOF_SEND;
  packet.frame_header.id = ID_GIMBAL_CMD;
  packet.frame_header.len = sizeof(SendGimbalCmdData) - 6U;
  packet.data.mode = 1;
  packet.data.yaw = 0.25F;
  crc8::append_CRC8_check_sum(reinterpret_cast<uint8_t *>(&packet), sizeof(HeaderFrame));
  crc16::append_CRC16_check_sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  auto bytes = toVector(packet);
  EXPECT_TRUE(crc8::verify_CRC8_check_sum(bytes.data(), sizeof(HeaderFrame)));
  EXPECT_TRUE(crc16::verify_CRC16_check_sum(bytes));
}

TEST(GimbalProtocol, BoundedDecodeDoesNotReadPastInput)
{
  std::vector<uint8_t> short_frame(1U, 0xFFU);
  const auto packet = fromVector<ReceiveGimbalStateData>(short_frame);
  EXPECT_EQ(packet.frame_header.sof, 0xFFU);
  EXPECT_EQ(packet.frame_header.len, 0U);
  EXPECT_FLOAT_EQ(packet.data.q[0], 0.0F);
}

}  // namespace standard_robot_pp_ros2
