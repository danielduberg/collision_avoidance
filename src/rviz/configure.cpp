#include <collision_avoidance/configure.h>

#include <QLabel>

namespace collision_avoidance_rviz
{
CAPanel::CAPanel(QWidget* parent) : rviz::Panel(parent) {
  parameters_["robot_height"] = new QDoubleSpinBox();
  parameters_["robot_radius"] = new QDoubleSpinBox();
  parameters_["security_distance"] = new QDoubleSpinBox();
  parameters_["epsilon"] = new QDoubleSpinBox();
  parameters_["min_distance_hold"] = new QDoubleSpinBox();
  parameters_["closest_distance"] = new QDoubleSpinBox();
  parameters_["h_m"] = new QDoubleSpinBox();
  parameters_["min_change_in_direction"] = new QDoubleSpinBox();
  parameters_["max_change_in_direction"] = new QDoubleSpinBox();
  parameters_["min_opposite_direction"] = new QDoubleSpinBox();
  parameters_["max_opposite_direction"] = new QDoubleSpinBox();
  parameters_["max_xy_vel"] = new QDoubleSpinBox();
  parameters_["max_z_vel"] = new QDoubleSpinBox();
  parameters_["max_yaw_rate"] = new QDoubleSpinBox();

  robot_frame_ = new QLineEdit();
  robot_height_ = new QDoubleSpinBox();

  robot_radius_ = new QDoubleSpinBox();
  security_distance_ = new QDoubleSpinBox();
  epsilon_ = new QDoubleSpinBox();

  min_distance_hold_ = new QDoubleSpinBox();
  closest_distance_ = new QDoubleSpinBox();
  h_m_ = new QDoubleSpinBox();

  min_change_in_direction_ = new QDoubleSpinBox();
  max_change_in_direction_ = new QDoubleSpinBox();

  min_opposite_direction_ = new QDoubleSpinBox();
  max_opposite_direction_ = new QDoubleSpinBox();

  max_xy_vel_ = new QDoubleSpinBox();
  max_z_vel_ = new QDoubleSpinBox();
  max_yaw_rate_ = new QDoubleSpinBox();


  // Design
  line_.resize(5);
  for (size_t i = 0; i < line_.size(); ++i)
  {
    line_[i] = new QFrame();
    line_[i]->setGeometry(QRect(/* ... */));
    line_[i]->setFrameShape(QFrame::HLine); // Replace by VLine for vertical line
    line_[i]->setFrameShadow(QFrame::Sunken);
  }

  unsigned int line_index = 0;

  layout_ = new QGridLayout();
  layout_->addWidget(new QLabel("Robot frame:"), 0, 0);
  layout_->addWidget(robot_frame_, 0, 1);
  layout_->addWidget(new QLabel("Robot height:"), 1, 0);
  layout_->addWidget(robot_height_, 1, 1);
  layout_->addWidget(new QLabel("m"), 1, 2);

  if (line_index < line_.size())
  {
      layout_->addWidget(line_[line_index++], 2, 0, 1, 3);
  }

  layout_->addWidget(new QLabel("Robot Radius:"), 3, 0);
  layout_->addWidget(robot_radius_, 3, 1);
  layout_->addWidget(new QLabel("m"), 3, 2);
  layout_->addWidget(new QLabel("Security distance:"), 4, 0);
  layout_->addWidget(security_distance_, 4, 1);
  layout_->addWidget(new QLabel("m"), 4, 2);
  layout_->addWidget(new QLabel("Epsilon:"), 5, 0);
  layout_->addWidget(epsilon_, 5, 1);

  if (line_index < line_.size())
  {
      layout_->addWidget(line_[line_index++], 6, 0, 1, 3);
  }

  layout_->addWidget(new QLabel("Min distance hold:"), 7, 0);
  layout_->addWidget(min_distance_hold_, 7, 1);
  layout_->addWidget(new QLabel("m"), 7, 2);
  layout_->addWidget(new QLabel("Closest distance:"), 8, 0);
  layout_->addWidget(closest_distance_, 8, 1);
  layout_->addWidget(new QLabel("m"), 8, 2);
  layout_->addWidget(new QLabel("h_m:"), 9, 0);
  layout_->addWidget(h_m_, 9, 1);

  if (line_index < line_.size())
  {
      layout_->addWidget(line_[line_index++], 10, 0, 1, 3);
  }

  layout_->addWidget(new QLabel("Max direction change:"), 11, 0);
  layout_->addWidget(new QLabel("deg"), 11, 2);
  layout_->addWidget(new QLabel("Min direction change:"), 12, 0);
  layout_->addWidget(new QLabel("deg"), 12, 2);

  if (line_index < line_.size())
  {
      layout_->addWidget(line_[line_index++], 13, 0, 1, 3);
  }

  layout_->addWidget(new QLabel("Min opposite direction:"), 14, 0);
  layout_->addWidget(new QLabel("deg"), 14, 2);
  layout_->addWidget(new QLabel("Max opposite direction:"), 15, 0);
  layout_->addWidget(new QLabel("deg"), 15, 2);

  if (line_index < line_.size())
  {
      layout_->addWidget(line_[line_index++], 16, 0, 1, 3);
  }

  layout_->addWidget(new QLabel("Max xy velocity:"), 17, 0);
  layout_->addWidget(max_xy_vel_, 17, 1);
  layout_->addWidget(new QLabel("m/s"), 17, 2);
  layout_->addWidget(new QLabel("Max z velocity:"), 18, 0);
  layout_->addWidget(max_z_vel_, 18, 1);
  layout_->addWidget(new QLabel("m/s"), 18, 2);
  layout_->addWidget(new QLabel("Max yaw rate:"), 19, 0);
  layout_->addWidget(max_yaw_rate_, 19, 1);
  layout_->addWidget(new QLabel("deg/s"), 19, 2);

  setLayout(layout_);
}

void CAPanel::load(const rviz::Config& config) {}

void CAPanel::save(rviz::Config config) const {}
}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(collision_avoidance_rviz::CAPanel, rviz::Panel)
