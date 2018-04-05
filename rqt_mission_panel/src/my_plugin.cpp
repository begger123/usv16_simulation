#include "rqt_mission_panel/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_mission_panel
{

MissionPanel::MissionPanel() : rqt_gui_cpp::Plugin(), widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MissionPanel");
}

void MissionPanel::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" +
                            QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);
}

void MissionPanel::shutdownPlugin()
{
  // unregister all publishers here
}

void MissionPanel::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MissionPanel::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_mission_panel::MissionPanel, rqt_gui_cpp::Plugin)
