#ifndef TWIST_WITH_COVARIANCE_DISPLAY_H
#define TWIST_WITH_COVARIANCE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <rviz/ogre_helpers/arrow.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;

class CovarianceProperty;
class CovarianceVisual;
}

namespace twist_with_covariance_rviz_plugin
{
    class TwistWithCovarianceDisplay : public rviz::MessageFilterDisplay<geometry_msgs::TwistWithCovarianceStamped>
    {
        Q_OBJECT
    public:
        TwistWithCovarianceDisplay();
        virtual ~TwistWithCovarianceDisplay();

        virtual void reset();
        virtual void onInitialize();

    protected:
        virtual void onEnable();

    private Q_SLOTS:
        void updateShapeVisibility();
        void updateColorAndAlpha();
        void updateArrowGeometry();

    private:
        void processMessage(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
        void updateArrow(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

        rviz::ColorProperty* linear_vel_color_property_;
        rviz::FloatProperty* linear_vel_alpha_property_;
        rviz::FloatProperty* linear_vel_scale_property_;
        rviz::FloatProperty* linear_vel_head_radius_property_;
        rviz::FloatProperty* linear_vel_shaft_radius_property_;

        rviz::ColorProperty* angular_vel_color_property_;
        rviz::FloatProperty* angular_vel_alpha_property_;
        rviz::FloatProperty* angular_vel_scale_property_;
        rviz::FloatProperty* angular_vel_head_radius_property_;
        rviz::FloatProperty* angular_vel_shaft_radius_property_;

        rviz::CovarianceProperty* linear_cov_property_;
        rviz::CovarianceProperty* angular_cov_property_;

        rviz::Arrow* linear_vel_arrow_;
        rviz::Arrow* angular_vel_arrow_;
        boost::shared_ptr<rviz::CovarianceVisual> linear_cov_;
        boost::shared_ptr<rviz::CovarianceVisual> angular_cov_;

        float latest_linear_vel;
        float latest_angular_vel;

        bool pose_valid_;
    
    };
}

#endif // TWIST_WITH_COVARIANCE_DISPLAY_H
