/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <boost/bind.hpp>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"
#include "rviz/ogre_helpers/shape.h"

#include "odometry_display.h"
#include "Eigen/Dense"

namespace rviz_plugin_odometry_covariance
{

OdometryCovarianceDisplay::OdometryCovarianceDisplay()
  : Display()
  , messages_received_(0)
{
  topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nav_msgs::Odometry>() ),
                                          "nav_msgs::Odometry topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  color_property_ = new rviz::ColorProperty( "Color", QColor( 255, 25, 0 ),
                                       "Color of the arrows.",
                                       this, SLOT( updateColor() ));

  position_tolerance_property_ = new rviz::FloatProperty( "Position Tolerance", .1,
                                                    "Distance, in meters from the last arrow dropped, "
                                                    "that will cause a new arrow to drop.",
                                                    this );
  position_tolerance_property_->setMin( 0 );
                                                
  angle_tolerance_property_ = new rviz::FloatProperty( "Angle Tolerance", .1,
                                                 "Angular distance from the last arrow dropped, "
                                                 "that will cause a new arrow to drop.",
                                                 this );
  angle_tolerance_property_->setMin( 0 );

  keep_property_ = new rviz::IntProperty( "Keep", 100,
                                    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
                                    this );
  keep_property_->setMin( 0 );

  length_property_ = new rviz::FloatProperty( "Length", 1.0,
                                        "Length of each arrow.",
                                        this, SLOT( updateLength() ));
}

OdometryCovarianceDisplay::~OdometryCovarianceDisplay()
{
  if ( initialized() )
  {
    unsubscribe();
    clear();
    delete tf_filter_;
  }
}

void OdometryCovarianceDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                          5, update_nh_ );

  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &OdometryCovarianceDisplay::incomingMessage, this, _1 ));
  context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
}

void OdometryCovarianceDisplay::clear()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  arrows_.clear();

  D_Ellipse::iterator eit = ellipses_.begin();
  D_Ellipse::iterator eend = ellipses_.end();
  for ( ; eit != eend; ++eit )
  {
    delete *eit;
  }
  ellipses_.clear();

  if( last_used_message_ )
  {
    last_used_message_.reset();
  }

  tf_filter_->clear();

  messages_received_ = 0;
  setStatus( rviz::StatusProperty::Warn, "Topic", "No messages received" );
}

void OdometryCovarianceDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
  context_->queueRender();
}

void OdometryCovarianceDisplay::updateColor()
{
  QColor color = color_property_->getColor();
  float red   = color.redF();
  float green = color.greenF();
  float blue  = color.blueF();

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for( ; it != end; ++it )
  {
    rviz::Arrow* arrow = *it;
    arrow->setColor( red, green, blue, 1.0f );
  }

  D_Ellipse::iterator eit = ellipses_.begin();
  D_Ellipse::iterator eend = ellipses_.end();
  for( ; eit != eend; ++eit )
  {
    rviz::Shape* ellipse = *eit;
    ellipse->setColor( red, green, blue, 1.0f );
  }
  context_->queueRender();
}

void OdometryCovarianceDisplay::updateLength()
{
  float length = length_property_->getFloat();
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  Ogre::Vector3 scale( length, length, length );
  for ( ; it != end; ++it )
  {
    rviz::Arrow* arrow = *it;
    arrow->setScale( scale );
  }
  context_->queueRender();
}

void OdometryCovarianceDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 5 );
    setStatus( rviz::StatusProperty::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( rviz::StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
  }
}

void OdometryCovarianceDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void OdometryCovarianceDisplay::onEnable()
{
  subscribe();
}

void OdometryCovarianceDisplay::onDisable()
{
  unsubscribe();
  clear();
}

bool validateFloats(const nav_msgs::Odometry& msg)
{
  bool valid = true;
  valid = valid && rviz::validateFloats( msg.pose.pose );
  valid = valid && rviz::validateFloats( msg.twist.twist );
  return valid;
}

void OdometryCovarianceDisplay::incomingMessage( const nav_msgs::Odometry::ConstPtr& message )
{
  ++messages_received_;

  if( !validateFloats( *message ))
  {
    setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  setStatus( rviz::StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

  if( last_used_message_ )
  {
    Ogre::Vector3 last_position(last_used_message_->pose.pose.position.x, last_used_message_->pose.pose.position.y, last_used_message_->pose.pose.position.z);
    Ogre::Vector3 current_position(message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z);
    Ogre::Quaternion last_orientation(last_used_message_->pose.pose.orientation.w, last_used_message_->pose.pose.orientation.x, last_used_message_->pose.pose.orientation.y, last_used_message_->pose.pose.orientation.z);
    Ogre::Quaternion current_orientation(message->pose.pose.orientation.w, message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z);

    if( (last_position - current_position).length() < position_tolerance_property_->getFloat() &&
        (last_orientation - current_orientation).normalise() < angle_tolerance_property_->getFloat() )
    {
      return;
    }
  }

  rviz::Arrow* arrow = new rviz::Arrow( scene_manager_, scene_node_, 0.8f, 0.05f, 0.2f, 0.2f );
  QColor color = color_property_->getColor();
  
  rviz::Shape* ellipse = NULL;
  if (message->pose.covariance[0] != 0) {
    ellipse = new rviz::Shape( rviz::Shape::Sphere, scene_manager_ );

    Eigen::Matrix2f covMatrix;
    covMatrix(0,0) = message->pose.covariance[0];
    covMatrix(0,1) = message->pose.covariance[1];
    covMatrix(1,0) = message->pose.covariance[6];
    covMatrix(1,1) = message->pose.covariance[7];
      
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);
    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    double angle = (atan2(eigVectors(1,0), eigVectors(0, 0)));
    double maj = std::max(.01, sqrt(eigValues[0]));
    double min = std::max(.01, sqrt(eigValues[1]));
    Ogre::Vector3 covScale( maj, .0001, min );
    ellipse->setScale( covScale );
    std::cout << "min " << min << " max " << maj << "angle " << angle << std::endl;
    std::cout << Ogre::Quaternion() << std::endl;
    std::cout << Ogre::Quaternion( 0, 0, cos(angle/2), sin(angle/2) ).getYaw() << std::endl;
    ellipse -> setOrientation( Ogre::Quaternion( cos(angle/2), 0, 0, sin(angle/2) ) * Ogre::Quaternion( Ogre::Degree(-90), Ogre::Vector3::UNIT_X ));
    //ellipse->setOrientation( Ogre::Quaternion( Ogre::Degree(angle), Ogre::Vector3::UNIT_Y) *
    //      Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_X ));
    ellipse->setColor( color.redF(), color.greenF(), color.blueF(), 0.4f );
    ellipses_.push_back(ellipse);
  }

  transformArrowAndEllipse( message, arrow, ellipse );
  arrow->setColor( color.redF(), color.greenF(), color.blueF(), 1.0f );


  float length = length_property_->getFloat();
  Ogre::Vector3 scale( length, length, length );
  arrow->setScale( scale );

  arrows_.push_back( arrow );

  last_used_message_ = message;
  context_->queueRender();
}

void OdometryCovarianceDisplay::transformArrowAndEllipse( const nav_msgs::Odometry::ConstPtr& message, rviz::Arrow* arrow, rviz::Shape* ellipse )
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose.pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming odometry '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  arrow->setPosition( position );
  if (ellipse != NULL) {
    std::cout << "NON NULL COV" << std::endl;
    ellipse->setPosition( position );
  }

  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));
}

void OdometryCovarianceDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
  clear();
}

void OdometryCovarianceDisplay::update( float wall_dt, float ros_dt )
{
  size_t keep = keep_property_->getInt();
  if( keep > 0 )
  {
    while( arrows_.size() > keep )
    {
      delete arrows_.front();
      arrows_.pop_front();
      delete ellipses_.front();
      ellipses_.pop_front();
    }
  }
}

void OdometryCovarianceDisplay::reset()
{
  Display::reset();
  clear();
}

void OdometryCovarianceDisplay::setTopic( const QString &topic, const QString &datatype )
{
  topic_property_->setString( topic );
}

} // namespace rviz_plugin_odom_covariance

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_odometry_covariance::OdometryCovarianceDisplay, rviz::Display )
