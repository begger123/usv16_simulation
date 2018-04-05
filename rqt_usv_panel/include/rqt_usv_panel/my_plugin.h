#ifndef rqt_usv_panel_my_plugin_H
#define rqt_usv_panel_my_plugin_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_mission_panel/ui_my_plugin.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <QWidget>
#include <usv16/maneuver_list.h>

namespace rqt_usv_panel
{

class UsvPanel : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  UsvPanel();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

// Comment in to signal that the plugin has a way to configure it
// bool hasConfiguration() const;
// void triggerConfiguration();

signals:
  void velReceived(const QString&);
  void posReceived(const QString&);

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  ros::Subscriber vel_subscriber_;
  ros::Subscriber pos_subscriber_;
  ros::Subscriber maneuver_subscriber_;

  QString speed_, angle_;
  QStringList maneuvers_;

  void velCallback(const geometry_msgs::TwistConstPtr&);
  void posCallback(const geometry_msgs::Pose2DConstPtr&);
  void maneuverCallback(const usv16::maneuver_listConstPtr&);
};
} // namespace rqt_usv_panel
#endif // rqt_usv_panel_my_plugin_H
