#include "pure_nmea_gga_conversion/nmea_gga_conversion.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NmeaGgaConversion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
