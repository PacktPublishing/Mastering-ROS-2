#include <circle_display_plugin/circle_display.hpp>


namespace rviz_circle_plugin {
  using rviz_common::properties::StatusProperty;


  void CircleDisplay::onInitialize() {
    MFDClass::onInitialize();
  
    point_shape_ =  std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Sphere, scene_manager_, scene_node_);
    // Create a BillboardLine to represent the circle's circumference
    circle_line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  
    // Set up properties for color and line width
    line_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Circle Color", QColor(255, 0, 0), "Color to draw the circle.", this, SLOT(updateStyleLine())); 
    line_width_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Line Width", 0.1, "Width of the circle line.", this, SLOT(updateStyleLine()));
    color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
        "Point Color", QColor(255, 0, 0), "Color to draw the point.", this, SLOT(updateStylePoint()));
    circle_width_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
        "Ceneter Width", 0.1, "Width to center point.", this, SLOT(updateStylePoint()));


        
    updateStylePoint();
    updateStyleLine();

  }

void CircleDisplay::processMessage(const circle_msgs::msg::Circle::ConstSharedPtr msg)
{
  RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);

  if( msg->radius <= 0.0 ) return;
  
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Error transforming from frame '" << msg->header.frame_id <<
        "' to frame '" << qPrintable(fixed_frame_) << "'");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  Ogre::Vector3 point_pos;
  point_pos.x = msg->x;
  point_pos.y = msg->y;
  point_shape_->setPosition(point_pos);   

  // Generate points for the circle
  const size_t num_segments = 100;  // Number of segments to approximate the circle
  const float radius = msg->radius;
  for (size_t i = 0; i <= num_segments; ++i) {
    float angle = static_cast<float>(i) * Ogre::Math::TWO_PI / static_cast<float>(num_segments);
    float x = msg->x + radius * std::cos(angle);
    float y = msg->y + radius * std::sin(angle);
    //std::cout << "For!" << std::endl; 
    circle_line_->addPoint(Ogre::Vector3(x, y, 0.0f));
  }  
}

void CircleDisplay::updateStylePoint(){
  
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  
  if( point_shape_ ) {
    point_shape_->setColor(color);
    float size = circle_width_property_->getFloat();
    point_shape_->setScale(Ogre::Vector3(size, size, size));
  }
}

void CircleDisplay::updateStyleLine(){
  if (circle_line_) {
    Ogre::ColourValue color = rviz_common::properties::qtToOgre(line_color_property_->getColor());
    circle_line_->setColor( color.r, color.g, color.b, 1 );
    float line_width = line_width_property_->getFloat();
    circle_line_->setLineWidth(line_width);
  }
}

}  // namespace rviz_circle_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_circle_plugin::CircleDisplay, rviz_common::Display)
