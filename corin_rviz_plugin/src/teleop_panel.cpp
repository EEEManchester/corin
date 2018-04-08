
#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include "teleop_panel.h"

namespace corin_rviz_plugin
{

// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Sets ROS parameters corresponding to certain actions
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  // , linear_velocity_( 0 )
  // , angular_velocity_( 0 )
{
  button_front_ = new QPushButton("Forward", this);
  button_back_ = new QPushButton("Backwards", this);
  button_left_ = new QPushButton("Left", this);
  button_right_ = new QPushButton("Right", this);
  button_bodypose_ = new QPushButton("Bodypose", this);
  button_w_transition_ = new QPushButton("Wall Transition", this);

  // Then create the control widget.
  // drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( button_front_ );
  layout->addWidget( button_back_ );
  layout->addWidget( button_left_ );
  layout->addWidget( button_right_ );
  layout->addWidget( button_bodypose_ );
  layout->addWidget( button_w_transition_ );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TeleopPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  // connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
  connect( button_front_, SIGNAL (released()), this, SLOT (handleButtonFront()));
  connect( button_back_, SIGNAL (released()), this, SLOT (handleButtonBack()));
  connect( button_left_, SIGNAL (released()), this, SLOT (handleButtonLeft()));
  connect( button_right_, SIGNAL (released()), this, SLOT (handleButtonRight()));
  connect( button_bodypose_, SIGNAL (released()), this, SLOT (handleButtonBodypose()));
  connect( button_w_transition_, SIGNAL (released()), this, SLOT (handleButtonWallTransition()));

  // Start the timer.
  output_timer->start( 100 );

}
void TeleopPanel::handleButtonFront()
 {
    // set parameter
    nh_.setParam("corin/walk_front", true);
 }
void TeleopPanel::handleButtonBack()
 {
    // set parameter
    nh_.setParam("corin/walk_back", true);
 }
 void TeleopPanel::handleButtonLeft()
 {
    // set parameter
    nh_.setParam("corin/walk_left", true);
 }
 void TeleopPanel::handleButtonRight()
 {
    // set parameter
    nh_.setParam("corin/walk_right", true);
 }
 void TeleopPanel::handleButtonBodypose()
 {
    // set parameter
    nh_.setParam("corin/bodypose", true);
 }
 void TeleopPanel::handleButtonWallTransition()
 {
    // set parameter
    nh_.setParam("corin/wall_transition", true);
 }
// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
// void TeleopPanel::sendVel()
// {
//   if( ros::ok() && velocity_publisher_ )
//   {
//     geometry_msgs::Twist msg;
//     msg.linear.x = linear_velocity_;
//     msg.linear.y = 0;
//     msg.linear.z = 0;
//     msg.angular.x = 0;
//     msg.angular.y = 0;
//     msg.angular.z = angular_velocity_;
//     velocity_publisher_.publish( msg );
//   }
// }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} // end namespace corin_rviz_plugin

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(corin_rviz_plugin::TeleopPanel,rviz::Panel )
// END_TUTORIAL
