#include "rqt_usv_panel/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <cmath>

namespace rqt_usv_panel
{

UsvPanel::UsvPanel() : rqt_gui_cpp::Plugin(), widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("UsvPanel");
}

void UsvPanel::initPlugin(qt_gui_cpp::PluginContext& context)
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

  // define the subscribers
  vel_subscriber_ =
      getNodeHandle().subscribe("/ship/vel", 10, &UsvPanel::velCallback, this);
  pos_subscriber_ =
      getNodeHandle().subscribe("/ship/pose", 10, &UsvPanel::posCallback, this);
  maneuver_subscriber_ = getNodeHandle().subscribe(
      "/ship/maneuvers", 100, &UsvPanel::maneuverCallback, this);

  connect(this, SIGNAL(velReceived(const QString&)), ui_.speedEdit,
          SLOT(setText(const QString&)));
  connect(this, SIGNAL(posReceived(const QString&)), ui_.angleEdit,
          SLOT(setText(const QString&)));
}

void UsvPanel::shutdownPlugin()
{
  // unregister all publishers here
  vel_subscriber_.shutdown();
  pos_subscriber_.shutdown();
}

void UsvPanel::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void UsvPanel::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void UsvPanel::velCallback(const geometry_msgs::TwistConstPtr& msg)
{
  speed_ = QString::number(msg->linear.x, 'f', 2);

  emit velReceived(speed_);
}

void UsvPanel::posCallback(const geometry_msgs::Pose2DConstPtr& msg)
{
  angle_ = QString::number(msg->theta * 180 / M_PI, 'f', 2);

  emit posReceived(angle_);
}

void UsvPanel::maneuverCallback(const usv16::maneuver_listConstPtr& msg)
{
  maneuvers_.clear();

  for (auto iter = msg->maneuver.cbegin(); iter != msg->maneuver.cend(); ++iter)
  {
    maneuvers_.push_back(QString::fromStdString(*iter));
  }

  ui_.goalListWidget->clear();

  foreach (QString s, maneuvers_)
    ui_.goalListWidget->addItem(new QListWidgetItem(s));
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_usv_panel::UsvPanel, rqt_gui_cpp::Plugin)
