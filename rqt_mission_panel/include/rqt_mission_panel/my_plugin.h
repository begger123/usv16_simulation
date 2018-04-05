#ifndef rqt_mission_panel_my_plugin_H
#define rqt_mission_panel_my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <rqt_mission_panel/ui_my_plugin.h>
#include <QWidget>

namespace rqt_mission_panel
{

class MissionPanel : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MissionPanel();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
};
} // namespace rqt_mission_panel
#endif // rqt_mission_panel_my_plugin_H
