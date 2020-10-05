#ifndef AUTOLABOR_GAZEBO_PLUGINS_AUTOLABOR_GPS_GAZEBO_PLUGINS_H
#define AUTOLABOR_GAZEBO_PLUGINS_AUTOLABOR_GPS_GAZEBO_PLUGINS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/SphericalCoordinates.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include "ignition/math/Vector3.hh"

typedef boost::shared_ptr<gazebo::common::SphericalCoordinates> SphericalCoordinatesPtr;

using namespace gazebo;

namespace autolabor_gazebo {

    class GPSPlugin : public ModelPlugin {
    public:
        GPSPlugin();

        ~GPSPlugin();

        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

        void Reset();

    protected:
        virtual void UpdateChild();

    private:
        physics::ModelPtr parent_;
        GazeboRosPtr gazebo_ros_;
        event::ConnectionPtr update_connection_;
        physics::LinkPtr gps_link_;

        std::string link_name_, frame_name_, topic_name_;
        double reference_latitude_, reference_longitude_, reference_heading_, reference_altitude_;
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
        SphericalCoordinatesPtr sphericalCoordinates_;

        bool sa_flag_;
        double non_sa_beta_, non_sa_theta_, sa_beta_, sa_theta_, noise_theta_;
        double x_dop_, y_dop_, v_dop_;
        ignition::math::Vector3<double> non_sa_error_, sa_error_;


        ros::Publisher fix_publisher_;
        sensor_msgs::NavSatFix fix_;

    private:
        template<typename T>
        static inline T randGaussian(T mu, T sigma) {
            T U = (T) random() / (T) RAND_MAX; // normalized uniform random variable
            T V = (T) random() / (T) RAND_MAX; // normalized uniform random variable
            T X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
            X = sigma * X + mu;
            return X;
        }
    };
}


#endif //AUTOLABOR_GAZEBO_PLUGINS_AUTOLABOR_GPS_GAZEBO_PLUGINS_H
