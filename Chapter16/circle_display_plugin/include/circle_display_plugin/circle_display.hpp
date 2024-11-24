#ifndef RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
#define RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_

#include <memory>
#include <rviz_common/message_filter_display.hpp>
#include <circle_msgs/msg/circle.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/selection_manager.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_rendering/objects/grid.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <OgreVector3.h>

#include <rviz_common/properties/float_property.hpp>

namespace rviz_circle_plugin
{
class CircleDisplay
  : public rviz_common::MessageFilterDisplay<circle_msgs::msg::Circle>
{
  Q_OBJECT
  private Q_SLOTS:
  void updateStyle();
  void updateStyleLine();


protected :
  void onInitialize() override;
  void processMessage(const circle_msgs::msg::Circle::ConstSharedPtr msg) override;
  std::unique_ptr<rviz_rendering::Shape> point_shape_;

  std::unique_ptr<rviz_rendering::BillboardLine> circle_line_;

  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
  std::unique_ptr<rviz_common::properties::ColorProperty> line_color_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> line_width_property_;
  std::unique_ptr<rviz_common::properties::FloatProperty> circle_width_property_;

};


}  // namespace rviz_circle_plugin

#endif  // RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
