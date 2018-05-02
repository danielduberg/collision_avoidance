#ifndef COLLISION_AVOIDANCE_RVIZ_CONFIGURE_H
#define COLLISION_AVOIDANCE_RVIZ_CONFIGURE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QGridLayout>
#include <QFrame>
#include <QSlider>
#include <QLineEdit>
#include <QDoubleSpinBox>

#include <map>

namespace collision_avoidance_rviz
{
class CAPanel : public rviz::Panel
{
  Q_OBJECT
public:
  CAPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:

  protected:
  QGridLayout* layout_;
  std::vector<QFrame*> line_;

  QLineEdit* robot_frame_;

  std::map<std::string, QDoubleSpinBox*> parameters_;

  QDoubleSpinBox* robot_height_;

  QDoubleSpinBox* robot_radius_;
  QDoubleSpinBox* security_distance_;
  QDoubleSpinBox* epsilon_;

  QDoubleSpinBox* min_distance_hold_;
  QDoubleSpinBox* closest_distance_;
  QDoubleSpinBox* h_m_;

  QDoubleSpinBox* min_change_in_direction_;
  QDoubleSpinBox* max_change_in_direction_;

  QDoubleSpinBox* min_opposite_direction_;
  QDoubleSpinBox* max_opposite_direction_;

  QDoubleSpinBox* max_xy_vel_;
  QDoubleSpinBox* max_z_vel_;
  QDoubleSpinBox* max_yaw_rate_;
};
}

#endif // COLLISION_AVOIDANCE_RVIZ_CONFIGURE_H
