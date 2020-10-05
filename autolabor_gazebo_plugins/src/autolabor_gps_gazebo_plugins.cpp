#include "autolabor_gazebo_plugins/autolabor_gps_gazebo_plugins.h"

namespace autolabor_gazebo {

    GPSPlugin::GPSPlugin() : non_sa_error_(0.0, 0.0, 0.0), sa_error_(0.0, 0.0, 0.0) {

    }

    GPSPlugin::~GPSPlugin() {

    }

    void GPSPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
        parent_ = parent;
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(parent, sdf, "GPSPlugin"));
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(link_name_, "linkName", "gps");
        gazebo_ros_->getParameter<std::string>(frame_name_, "frameName", "world");
        gazebo_ros_->getParameter<std::string>(topic_name_, "topicName", "gps");
        gazebo_ros_->getParameter<double>(reference_latitude_, "referenceLatitude", 0.0);
        gazebo_ros_->getParameter<double>(reference_longitude_, "referenceLongitude", 0.0);
        gazebo_ros_->getParameter<double>(reference_heading_, "referenceHeading", 0.0);
        gazebo_ros_->getParameter<double>(reference_altitude_, "referenceAltitude", 0.0);
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 10.0);

        gazebo_ros_->getParameter<bool>(sa_flag_, "SAFlag", true);
        gazebo_ros_->getParameter<double>(non_sa_beta_, "nonSABeta", 0.001);
        gazebo_ros_->getParameter<double>(non_sa_theta_, "nonSATheta", 0.02);
        gazebo_ros_->getParameter<double>(sa_beta_, "SABeta", 0.01);
        gazebo_ros_->getParameter<double>(sa_theta_, "SATheta", 2.0);
        gazebo_ros_->getParameter<double>(noise_theta_, "noiseTheta", 0.02);
        gazebo_ros_->getParameter<double>(x_dop_, "xDOP", 0.7);
        gazebo_ros_->getParameter<double>(y_dop_, "yDOP", 0.8);
        gazebo_ros_->getParameter<double>(v_dop_, "vDOP", 1.1);

        gps_link_ = parent_->GetLink(link_name_);
        if (!gps_link_) {
            ROS_FATAL("GPSPlugin error: gps_link_name: %s does not exist\n", link_name_.c_str());
            return;
        }

        sphericalCoordinates_ = SphericalCoordinatesPtr(
                new common::SphericalCoordinates(common::SphericalCoordinates::EARTH_WGS84,
                                                 ignition::math::Angle(reference_latitude_ * M_PI / 180.0),
                                                 ignition::math::Angle(reference_longitude_ * M_PI / 180.0),
                                                 reference_altitude_,
                                                 ignition::math::Angle(reference_heading_ * M_PI / 180.0))
        );

        if (update_rate_ > 0) {
            update_period_ = 1.0 / update_rate_;
        } else {
            update_period_ = 0.0;
        }
        last_update_time_ = parent_->GetWorld()->SimTime();

        fix_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::NavSatFix>(topic_name_, 10);
        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPSPlugin::UpdateChild, this));
    }

    void GPSPlugin::UpdateChild() {
        common::Time current_time = parent_->GetWorld()->SimTime();
        double dt = (current_time - last_update_time_).Double();
        if (dt >= update_period_) {
            last_update_time_ = current_time;
            ignition::math::Vector3<double> pose = gps_link_->WorldPose().Pos();
            for (int i = 0; i < 3; i++) {
                non_sa_error_[i] = non_sa_error_[i] * exp(-non_sa_beta_ * dt) + randGaussian(0.0, non_sa_theta_);
                sa_error_[i] = sa_error_[i] * exp(sa_beta_ * dt) + randGaussian(0.0, sa_theta_);
            }

            ignition::math::Vector3<double> noise_pose;
            if (sa_flag_) {
                noise_pose = (non_sa_error_ + sa_error_ +
                              ignition::math::Vector3<double>(randGaussian(pose.X(), noise_theta_),
                                                              randGaussian(pose.Y(), noise_theta_),
                                                              randGaussian(pose.Z(), noise_theta_))) *
                             ignition::math::Vector3<double>(x_dop_, y_dop_, v_dop_);
            } else {
                noise_pose = (non_sa_error_ +
                              ignition::math::Vector3<double>(randGaussian(pose.X(), noise_theta_),
                                                              randGaussian(pose.Y(), noise_theta_),
                                                              randGaussian(pose.Z(), noise_theta_))) *
                             ignition::math::Vector3<double>(x_dop_, y_dop_, v_dop_);
            }

            ignition::math::Vector3<double> spherical = sphericalCoordinates_->SphericalFromLocal(noise_pose);

            fix_.header.frame_id = frame_name_;
            fix_.header.stamp = ros::Time(current_time.sec, current_time.nsec);
            fix_.latitude = spherical.X();
            fix_.longitude = spherical.Y();
            fix_.altitude = spherical.Z();

            fix_publisher_.publish(fix_);
        }
    }

    void GPSPlugin::Reset() {
        last_update_time_ = parent_->GetWorld()->SimTime();
        non_sa_error_ = ignition::math::Vector3<double>::Zero;
        sa_error_ = ignition::math::Vector3<double>::Zero;
    }

    GZ_REGISTER_MODEL_PLUGIN (GPSPlugin)
}
