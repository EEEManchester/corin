
#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QString>
#include <QIcon>

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
  , cmd_state_("hold")
  , run_toggle_(false)
  , jump_toggle_(false)
  // , motion_selected_( false )
  // , angular_velocity_( 0 )
{
  //***************** PUBLISHERS ***************//
  ui_publisher_ = nh_.advertise<std_msgs::String>( "/corin/ui_execute", 1 );

  //***************** SERVICES ***************//
  ui_client_ = nh_.serviceClient<corin_msgs::UiState>("/corin/set_ui_state");

  button_front_ = new QPushButton("Forward", this);
  button_back_ = new QPushButton("Backward", this);
  button_left_ = new QPushButton("Left", this);
  button_right_ = new QPushButton("Right", this);
  button_bodypose_ = new QPushButton("Bodypose", this);
  button_rotate_ = new QPushButton("Rotate", this);
  button_reset_ = new QPushButton("Reset", this);

  tran_group_ = TransitionGroup();
  exec_group_ = ExecutionGroup();

  hline1_ = new QFrame;
  hline1_ -> setFrameShape(QFrame::HLine);
  hline2_ = new QFrame;
  hline2_ -> setFrameShape(QFrame::HLine);
  hline3_ = new QFrame;
  hline3_ -> setFrameShape(QFrame::HLine);

  // Then create the control widget.
  // drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* main_layout  = new QVBoxLayout;
  
  QGridLayout *poses_layout = new QGridLayout;
  poses_layout->addWidget( button_bodypose_, 0, 0 );
  poses_layout->addWidget( button_rotate_ , 0, 1);

  QGridLayout *direction_layout = new QGridLayout;
  direction_layout->addWidget( button_front_, 0, 1 );
  direction_layout->addWidget( button_back_, 2, 1 );
  direction_layout->addWidget( button_left_, 1, 0 );
  direction_layout->addWidget( button_reset_, 1, 1 );
  direction_layout->addWidget( button_right_, 1, 2 );
  
  main_layout->addLayout(poses_layout);
  main_layout->addWidget(hline1_ );
  main_layout->addLayout(direction_layout);
  main_layout->addWidget(hline2_ );
  main_layout->addWidget(tran_group_);
  main_layout->addWidget(hline3_ );
  main_layout->addWidget(exec_group_);
  
  setLayout( main_layout );

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
  // connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendUserCmd() ));

  connect( button_front_, SIGNAL (released()), this, SLOT (handleButtonFront()));
  connect( button_back_, SIGNAL (released()), this, SLOT (handleButtonBack()));
  connect( button_left_, SIGNAL (released()), this, SLOT (handleButtonLeft()));
  connect( button_right_, SIGNAL (released()), this, SLOT (handleButtonRight()));
  connect( button_bodypose_, SIGNAL (released()), this, SLOT (handleButtonBodypose()));
  connect( button_rotate_, SIGNAL (released()), this, SLOT (handleButtonRotate()));
  connect( button_g2w_transition_, SIGNAL (released()), this, SLOT (handleButtonGnd2WallTransition()));
  connect( button_w2g_transition_, SIGNAL (released()), this, SLOT (handleButtonWall2GndTransition()));
  connect( button_g2c_transition_, SIGNAL (released()), this, SLOT (handleButtonGnd2ChimneyTransition()));
  connect( button_c2g_transition_, SIGNAL (released()), this, SLOT (handleButtonChimney2GndTransition()));

  connect( button_execute_, SIGNAL (released()), this, SLOT (handleButtonExecute()));
  connect( button_cancel_, SIGNAL (released()), this, SLOT (handleButtonCancel()));
  connect( button_pose_jump_, SIGNAL (released()), this, SLOT (handleButtonPoseJump()));

  // Start the timer.
  // output_timer->start( 500 );

}
QGroupBox *TeleopPanel::TransitionGroup()
{
    QGroupBox *groupBox = new QGroupBox(tr("Transitions"));

    // groupBox->setStyleSheet("margin: 1px solid black");
    button_g2w_transition_ = new QPushButton("Gnd to Wall", this);
    button_w2g_transition_ = new QPushButton("Wall to Gnd", this);
    button_g2c_transition_ = new QPushButton("Gnd to Chimney", this);
    button_c2g_transition_ = new QPushButton("Chimney to Gnd", this);

    QGridLayout *transition_layout = new QGridLayout;
    transition_layout->addWidget( button_g2w_transition_,0 ,1 );
    transition_layout->addWidget( button_w2g_transition_,0 ,2 );
    transition_layout->addWidget( button_g2c_transition_,1 ,1 );
    transition_layout->addWidget( button_c2g_transition_,1 ,2 );

    groupBox->setLayout(transition_layout);
    // groupBox->setFlat(false);
    return groupBox;
}

QGroupBox *TeleopPanel::ExecutionGroup()
{
    QGroupBox *groupBox = new QGroupBox(tr(""));

    // groupBox->setStyleSheet("margin: 1px solid black");

    button_execute_ = new QPushButton("Run", this);
    button_cancel_  = new QPushButton("Cancel", this);
    button_pose_jump_ = new QPushButton("End Pose", this);

    QGridLayout* grid_layout  = new QGridLayout;
    grid_layout->addWidget( button_execute_, 0, 0);
    grid_layout->addWidget( button_cancel_, 0, 1);
    grid_layout->addWidget( button_pose_jump_, 1, 0, 1, 2);

    groupBox->setLayout(grid_layout);
    // groupBox->setFlat(false);
    return groupBox;
}

void TeleopPanel::handleButtonFront()
 {
    enableExecButtons(true);
    nh_.setParam("corin/walk_front", true);
 }
void TeleopPanel::handleButtonBack()
 {
    enableExecButtons(true);
    nh_.setParam("corin/walk_back", true);
 }
 void TeleopPanel::handleButtonLeft()
 {
    QString utext = button_left_->text();

    if (utext == "Left")
      nh_.setParam("corin/walk_left", true);
    else if (utext == "Upward")
      nh_.setParam("corin/walk_up", true);

    enableExecButtons(true);
 }
 void TeleopPanel::handleButtonRight()
 {
    QString utext = button_right_->text();
    
    if (utext == "Right")
      nh_.setParam("corin/walk_right", true);
    else if (utext == "Downward")
      nh_.setParam("corin/walk_down", true);

    enableExecButtons(true);
 }
 void TeleopPanel::handleButtonBodypose()
 {
    enableExecButtons(true);
    nh_.setParam("corin/bodypose", true);
 }
 void TeleopPanel::handleButtonRotate()
 {
    enableExecButtons(true);
    nh_.setParam("corin/rotate", true);
 }
 void TeleopPanel::handleButtonGnd2WallTransition()
 {
    enableExecButtons(true);
    nh_.setParam("corin/g2w_transition", true);
 }
 void TeleopPanel::handleButtonWall2GndTransition()
 {
    enableExecButtons(true);
    nh_.setParam("corin/w2g_transition", true);
 }
 void TeleopPanel::handleButtonGnd2ChimneyTransition()
 {
    enableExecButtons(true);
    nh_.setParam("corin/g2c_transition", true);
    button_left_->setText("Upward");
    button_right_->setText("Downward");
 }
 void TeleopPanel::handleButtonChimney2GndTransition()
 {
    enableExecButtons(true);
    nh_.setParam("corin/c2g_transition", true);
    button_left_->setText("Left");
    button_right_->setText("Right");
 }
// ====================================================== //
// ====================================================== //
 void TeleopPanel::handleButtonExecute()
 {
    if (run_toggle_ == false)
    {
      cmd_state_  = "play";
      run_toggle_ = true;
      button_execute_->setText("Pause");
      // ROS_INFO("should run");
    }
    else if (run_toggle_ == true)
    {
      cmd_state_  = "pause";
      run_toggle_ = false;
      button_execute_->setText("Run");
      ROS_INFO("should pause");
    }
    sendUserCmd(cmd_state_);
    
 }
 void TeleopPanel::handleButtonCancel()
 {
    cmd_state_ = "cancel";
    enableExecButtons(false);
    sendUserCmd(cmd_state_);
 }
 void TeleopPanel::enableExecButtons(bool mselect)
 {
  // disable buttons when no motion selected, or cancelled
  if (mselect == false)
  {
    button_execute_->setEnabled(false);
    button_cancel_->setEnabled(false);
  }
  // motion has been selected, enable run/cancel buttons
  else if (mselect == true)
  {
    button_execute_->setEnabled(true);
    button_cancel_->setEnabled(true);
    button_execute_->setText("Run");
    run_toggle_ = false;
  }
 }

 void TeleopPanel::handleButtonPoseJump()
 {
  if (jump_toggle_ == false)
  {
    if (sendSrvCmd("go_final_pose"))
    {
      jump_toggle_ = true;
      button_pose_jump_->setText("Initial Pose"); 
      ROS_INFO("Moving robot to final pose"); 
    }
  }
  else if (jump_toggle_ == true)
  {
    if (sendSrvCmd("go_initial_pose"))
    {
      jump_toggle_ = false;
      button_pose_jump_->setText("Final Pose");
      ROS_INFO("Moving robot to initial pose"); 
    }
  }
 }
// Sends the command to service function
bool TeleopPanel::sendSrvCmd(std::string srv_cmd)
{
  std_msgs::String msg;
  msg.data = srv_cmd;
  
  corin_msgs::UiState srv;
  srv.request.state = msg;
  
  if (ui_client_.call(srv))
  {
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return false;
  }
}

// Publish the commanded state if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void TeleopPanel::sendUserCmd(std::string cmd_state_)
{
  if( ros::ok() && ui_publisher_ )
  {
    std_msgs::String msg;
    msg.data = cmd_state_;
    ui_publisher_.publish(msg);
  }
  // resets command
  cmd_state_ = "hold";
}

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
