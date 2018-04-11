
#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <QPushButton>
# include <QFrame>
# include <QGroupBox>
# include <rviz/panel.h>
#endif

namespace corin_rviz_plugin
{

// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class TeleopPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  TeleopPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  void handleButtonFront();
  void handleButtonBack();
  void handleButtonLeft();
  void handleButtonRight();
  void handleButtonBodypose();
  void handleButtonRotate();
  void handleButtonGnd2WallTransition();
  void handleButtonWall2GndTransition();
  void handleButtonChimneyTransition();

  void handleButtonExecute();
  void handleButtonCancel();
  void enableExecButtons(bool mselect);

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  // void setTopic( const QString& topic );

  // Here we declare some internal slots.
protected Q_SLOTS:
  
  // Then we finish up with protected member variables.
protected:
  QPushButton* button_front_;
  QPushButton* button_back_;
  QPushButton* button_left_;
  QPushButton* button_right_;
  QPushButton* button_bodypose_;
  QPushButton* button_rotate_;
  QPushButton* button_w2g_transition_;
  QPushButton* button_g2w_transition_;
  QPushButton* button_c_transition_;

  QPushButton* button_execute_;
  QPushButton* button_cancel_;

  QGroupBox* exec_group_;

  QFrame* hline1_;
  QFrame* hline2_;
  QFrame* hline3_;

  QGroupBox* ExecutionGroup();
  // The ROS publisher for the command velocity.
  // ros::Publisher velocity_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // The latest velocity values from the drive widget.
  // float linear_velocity_;
  // float angular_velocity_;
  // bool motion_selected_;

};

} // end namespace corin_rviz_plugin

#endif // TELEOP_PANEL_H
