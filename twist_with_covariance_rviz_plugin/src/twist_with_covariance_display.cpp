#include <Eigen/Core>
#include <Eigen/Dense>

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include "twist_with_covariance_display.h"

#include <rviz/default_plugin/covariance_property.h>
#include <rviz/default_plugin/covariance_visual.h>
#include <rviz/default_plugin/pose_with_covariance_display.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include <pluginlib/class_list_macros.h>

#include <rviz/validate_floats.h>

namespace twist_with_covariance_rviz_plugin {
    TwistWithCovarianceDisplay::TwistWithCovarianceDisplay() : pose_valid_(false), latest_linear_vel(0.0f), latest_angular_vel(0.0f){
        linear_vel_color_property_ = new rviz::ColorProperty("Linear Velocity Color", QColor(204, 51, 204),
                                                             "Color to draw the linear velocity arrow.",
                                                             this, SLOT(updateColorAndAlpha()));
        linear_vel_alpha_property_ = new rviz::FloatProperty("Linear Velocity Alpha", 1.0,
                                                             "0 is fully transparent, 1.0 is fully opaque.",
                                                             this, SLOT(updateColorAndAlpha()));
        linear_vel_scale_property_ = new rviz::FloatProperty("Linear Velocity Scale", 1.0,
                                                             "Scale of the linear velocity arrow.",
                                                             this, SLOT(updateArrowGeometry()));
        linear_vel_head_radius_property_ = new rviz::FloatProperty("Linear Velocity Head Radius", 0.1,
                                                                   "Radius of the arrow head.",
                                                                   this, SLOT(updateArrowGeometry()));
        linear_vel_shaft_radius_property_ = new rviz::FloatProperty("Linear Velocity Shaft Radius", 0.05,
                                                                    "Radius of the arrow shaft.",
                                                                    this, SLOT(updateArrowGeometry()));

        angular_vel_color_property_ = new rviz::ColorProperty("Angular Velocity Color", QColor(204, 51, 204),
                                                              "Color to draw the angular velocity arrow.",
                                                              this, SLOT(updateColorAndAlpha()));
        angular_vel_alpha_property_ = new rviz::FloatProperty("Angular Velocity Alpha", 1.0,
                                                              "0 is fully transparent, 1.0 is fully opaque.",
                                                              this, SLOT(updateColorAndAlpha()));
        angular_vel_scale_property_ = new rviz::FloatProperty("Angular Velocity Scale", 1.0,
                                                              "Scale of the angular velocity arrow.",
                                                              this, SLOT(updateArrowGeometry()));
        angular_vel_head_radius_property_ = new rviz::FloatProperty("Angular Velocity Head Radius", 0.1,
                                                                    "Radius of the arrow head.",
                                                                    this, SLOT(updateArrowGeometry()));
        angular_vel_shaft_radius_property_ = new rviz::FloatProperty("Angular Velocity Shaft Radius", 0.05,
                                                                     "Radius of the arrow shaft.",
                                                                     this, SLOT(updateArrowGeometry()));

        linear_cov_property_ = new rviz::CovarianceProperty(QString("Linear Covariance"), true, QString("Show the linear covariance."),
                                                            this, SLOT(updateShapeVisibility()));
        angular_cov_property_ = new rviz::CovarianceProperty(QString("Angular Covariance"), true, QString("Show the angular covariance."),
                                                             this, SLOT(updateShapeVisibility()));
    }

    void TwistWithCovarianceDisplay::onInitialize() {
        MFDClass::onInitialize();

        linear_vel_arrow_ = new rviz::Arrow(scene_manager_, scene_node_, 0.0f, linear_vel_shaft_radius_property_->getFloat(), 0.0f, linear_vel_head_radius_property_->getFloat());
        angular_vel_arrow_ = new rviz::Arrow(scene_manager_, scene_node_, 0.0f, angular_vel_shaft_radius_property_->getFloat(), 0.0f, angular_vel_head_radius_property_->getFloat());

        linear_cov_ = linear_cov_property_->createAndPushBackVisual(scene_manager_, scene_node_);
        angular_cov_ = angular_cov_property_->createAndPushBackVisual(scene_manager_, scene_node_);

        updateColorAndAlpha();
        updateArrowGeometry();
    }

    TwistWithCovarianceDisplay::~TwistWithCovarianceDisplay() {
        if (initialized()) {
            delete linear_vel_arrow_;
            delete angular_vel_arrow_;
        }
    }


    void TwistWithCovarianceDisplay::reset() {
        MFDClass::reset();
        linear_cov_property_->clearVisual();
        angular_cov_property_->clearVisual();
    }

    void TwistWithCovarianceDisplay::onEnable() {
        MFDClass::onEnable();
        linear_cov_property_->updateVisibility();
        angular_cov_property_->updateVisibility();
    }

    void TwistWithCovarianceDisplay::updateColorAndAlpha() {
        Ogre::ColourValue linear_vel_color = linear_vel_color_property_->getOgreColor();
        linear_vel_color.a = linear_vel_alpha_property_->getFloat();
        linear_vel_arrow_->setColor(linear_vel_color.r, linear_vel_color.g, linear_vel_color.b, linear_vel_color.a);

        Ogre::ColourValue angular_vel_color = angular_vel_color_property_->getOgreColor();
        angular_vel_color.a = angular_vel_alpha_property_->getFloat();
        angular_vel_arrow_->setColor(angular_vel_color.r, angular_vel_color.g, angular_vel_color.b, angular_vel_color.a);

        context_->queueRender();
    }

    void TwistWithCovarianceDisplay::updateArrowGeometry() {
        linear_vel_arrow_->set(latest_linear_vel * linear_vel_scale_property_->getFloat() * 0.7, linear_vel_shaft_radius_property_->getFloat(), latest_linear_vel * linear_vel_scale_property_->getFloat() * 0.3, linear_vel_head_radius_property_->getFloat());
        angular_vel_arrow_->set(latest_angular_vel * angular_vel_scale_property_->getFloat() * 0.7, angular_vel_shaft_radius_property_->getFloat(), latest_angular_vel * angular_vel_scale_property_->getFloat() * 0.3, angular_vel_head_radius_property_->getFloat());
    }

    void TwistWithCovarianceDisplay::updateShapeVisibility() {
        if (!pose_valid_) {
            linear_vel_arrow_->getSceneNode()->setVisible(false);
            angular_vel_arrow_->getSceneNode()->setVisible(false);
            linear_cov_->setVisible(false);
            angular_cov_->setVisible(false);
        } else {
            linear_vel_arrow_->getSceneNode()->setVisible(true);
            angular_vel_arrow_->getSceneNode()->setVisible(true);
            linear_cov_property_->updateVisibility();
            angular_cov_property_->updateVisibility();
        }
    }

    void TwistWithCovarianceDisplay::processMessage(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg) {
        // validate float values
        if (!rviz::validateFloats(msg->twist.twist.linear) ||
            !rviz::validateFloats(msg->twist.twist.angular) ||
            !rviz::validateFloats(msg->twist.covariance)) {
            setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
            return;
        }

        Ogre::Vector3 linear_vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        Ogre::Vector3 angular_vel(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        
        // validate transform
        if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)) {
            ROS_ERROR("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
            return;
        }

        pose_valid_ = true;
        updateShapeVisibility();

        // save linear and angular velocities
        latest_linear_vel = linear_vel.length();
        latest_angular_vel = angular_vel.length();

        // update arrow orientation & direction
        linear_vel_arrow_->setDirection(linear_vel);
        
        angular_vel_arrow_->setDirection(angular_vel);
        

        // update arrow geometry
        updateArrowGeometry();

        // prepare covariance
        geometry_msgs::PoseWithCovariance linear_cov_msg;
        geometry_msgs::PoseWithCovariance angular_cov_msg;
        Eigen::Matrix<double, 6, 6> covariance_matrix = Eigen::Map<const Eigen::Matrix<double, 6, 6> >(msg->twist.covariance.data());
        Eigen::Matrix<double, 6, 6> linear_covariance_matrix = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> angular_covariance_matrix = Eigen::Matrix<double, 6, 6>::Zero();
        linear_covariance_matrix.block<3, 3>(0, 0) = covariance_matrix.block<3, 3>(0, 0);
        angular_covariance_matrix.block<3, 3>(0, 0) = covariance_matrix.block<3, 3>(3, 3);

        // copy covariances to messages
        std::copy(linear_covariance_matrix.data(), linear_covariance_matrix.data() + linear_covariance_matrix.size(), linear_cov_msg.covariance.begin());
        std::copy(angular_covariance_matrix.data(), angular_covariance_matrix.data() + angular_covariance_matrix.size(), angular_cov_msg.covariance.begin());

        linear_cov_->setPosition(position);
        linear_cov_->setOrientation(orientation);
        linear_cov_->setCovariance(linear_cov_msg);

        angular_cov_->setPosition(position);
        angular_cov_->setOrientation(orientation);
        angular_cov_->setCovariance(angular_cov_msg);
        
        context_->queueRender();
        
    }
} // namespace twist_with_covariance_rviz_plugin

PLUGINLIB_EXPORT_CLASS(twist_with_covariance_rviz_plugin::TwistWithCovarianceDisplay, rviz::Display)