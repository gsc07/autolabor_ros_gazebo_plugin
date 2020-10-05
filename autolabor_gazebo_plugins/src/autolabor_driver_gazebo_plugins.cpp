#include <algorithm>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

#include "autolabor_gazebo_plugins/autolabor_driver_gazebo_plugins.h"

namespace autolabor_gazebo {

    enum {
        LEFT_FRONT,
        LEFT_BACK,
        RIGHT_FRONT,
        RIGHT_BACK,
    };

    enum {
        LEFT,
        RIGHT,
    };

    FourWheelDiffDriver::FourWheelDiffDriver() {}

    // Destructor
    FourWheelDiffDriver::~FourWheelDiffDriver() {
        FiniChild();
    }

    // Load the controller
    void FourWheelDiffDriver::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent_ = _parent;
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "FourWheelDiffDrive"));
        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_vel");
        gazebo_ros_->getParameter<std::string>(odometry_topic_, "odometryTopic", "odom");
        gazebo_ros_->getParameter<std::string>(odometry_frame_, "odometryFrame", "odom");
        gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_link");
        gazebo_ros_->getParameter<std::string>(world_frame_, "worldFrame", "world");

        gazebo_ros_->getParameterBoolean(publishWheelTF_, "publishWheelTF", false);
        gazebo_ros_->getParameterBoolean(publishOdomTF_, "publishOdomTF", true);
        gazebo_ros_->getParameterBoolean(publishWorldTF_, "publishWorldTF", true);
        gazebo_ros_->getParameterBoolean(publishWheelJointState_, "publishWheelJointState", false);
        gazebo_ros_->getParameterBoolean(legacy_mode_, "legacyMode", false);

        if (!_sdf->HasElement("legacyMode")) {
            ROS_ERROR_NAMED("four_wheel_diff_drive",
                            "FourWheelDiffDriver Plugin missing <legacyMode>, defaults to true\n"
                            "This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
                            "To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
                            "To fix an old package you have to exchange left wheel by the right wheel.\n"
                            "If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
                            "just set <legacyMode> to true.\n"
            );
        }

        gazebo_ros_->getParameter<double>(wheel_separation_, "wheelSeparation", 0.80);
        gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.25);
        gazebo_ros_->getParameter<double>(wheel_accel_, "wheelAcceleration", 0.0);
        gazebo_ros_->getParameter<double>(wheel_torque_, "wheelTorque", 50.0);
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 10.0);

        joints_.resize(4);
        joints_[LEFT_FRONT] = gazebo_ros_->getJoint(parent_, "leftFrontWheelJoint", "joint_left_front");
        joints_[LEFT_BACK] = gazebo_ros_->getJoint(parent_, "leftBackWheelJoint", "joint_left_back");
        joints_[RIGHT_FRONT] = gazebo_ros_->getJoint(parent_, "rightFrontWheelJoint", "joint_right_front");
        joints_[RIGHT_BACK] = gazebo_ros_->getJoint(parent_, "rightBackWheelJoint", "joint_right_back");


        joints_[LEFT_FRONT]->SetParam("fmax", 0, wheel_torque_);
        joints_[LEFT_BACK]->SetParam("fmax", 0, wheel_torque_);
        joints_[RIGHT_FRONT]->SetParam("fmax", 0, wheel_torque_);
        joints_[RIGHT_BACK]->SetParam("fmax", 0, wheel_torque_);



        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent_->GetWorld()->SimTime();
#else
        last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

        // Initialize velocity stuff
        wheel_speed_[LEFT] = 0;
        wheel_speed_[RIGHT] = 0;

        // Initialize velocity support stuff
        wheel_speed_instr_[LEFT] = 0;
        wheel_speed_instr_[RIGHT] = 0;


        x_ = 0;
        rot_ = 0;
        alive_ = true;


        if (this->publishWheelJointState_) {
            joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
            ROS_INFO_NAMED("four_wheel_diff_drive", "%s: Advertise joint_states", gazebo_ros_->info());
        }

        transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        ROS_INFO_NAMED("four_wheel_diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(),
                       command_topic_.c_str());

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                                                    boost::bind(
                                                                            &FourWheelDiffDriver::cmdVelCallback,
                                                                            this, _1),
                                                                    ros::VoidPtr(), &queue_);

        cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
        ROS_INFO_NAMED("four_wheel_diff_drive", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

        odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);


        // start custom queue for diff drive
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&FourWheelDiffDriver::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&FourWheelDiffDriver::UpdateChild, this));

    }

    void FourWheelDiffDriver::Reset() {
#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent_->GetWorld()->SimTime();
#else
        last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
        pose_encoder_.x = 0;
        pose_encoder_.y = 0;
        pose_encoder_.theta = 0;
        x_ = 0;
        rot_ = 0;
        joints_[LEFT_FRONT]->SetParam("fmax", 0, wheel_torque_);
        joints_[LEFT_BACK]->SetParam("fmax", 0, wheel_torque_);
        joints_[RIGHT_FRONT]->SetParam("fmax", 0, wheel_torque_);
        joints_[RIGHT_BACK]->SetParam("fmax", 0, wheel_torque_);
    }

    void FourWheelDiffDriver::publishWheelJointState() {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(joints_.size());
        joint_state_.position.resize(joints_.size());

        for (int i = 0; i < 4; i++) {
            physics::JointPtr joint = joints_[i];
#if GAZEBO_MAJOR_VERSION >= 8
            double position = joint->Position(0);
#else
            double position = joint->GetAngle ( 0 ).Radian();
#endif
            joint_state_.name[i] = joint->GetName();
            joint_state_.position[i] = position;
        }
        joint_state_publisher_.publish(joint_state_);
    }

    void FourWheelDiffDriver::publishWheelTF() {
        ros::Time current_time = ros::Time::now();
        for (int i = 0; i < 4; i++) {

            std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName());
            std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName());

#if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
#else
            ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

            tf::Quaternion qt(poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W());
            tf::Vector3 vt(poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z());

            tf::Transform tfWheel(qt, vt);
            transform_broadcaster_->sendTransform(
                    tf::StampedTransform(tfWheel, current_time, wheel_parent_frame, wheel_frame));
        }
    }

    // Update the controller
    void FourWheelDiffDriver::UpdateChild() {

        /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
           https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
           (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
           and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque_ other than FourWheelDiffDriver::Reset
           (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
        */
        for (int i = 0; i < 4; i++) {
            if (fabs(wheel_torque_ - joints_[i]->GetParam("fmax", 0)) > 1e-6) {
                joints_[i]->SetParam("fmax", 0, wheel_torque_);
            }
        }


        UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = parent_->GetWorld()->SimTime();
#else
        common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            publishOdometry();
            if (publishOdomTF_) publishOdomTF();
            if (publishWorldTF_) publishWorldTF();
            if (publishWheelTF_) publishWheelTF();
            if (publishWheelJointState_) publishWheelJointState();

            // Update robot in case new velocities have been requested
            getWheelVelocities();

            double current_speed[2];

            current_speed[LEFT] = joints_[LEFT]->GetVelocity(0) * (wheel_diameter_ / 2.0);
            current_speed[RIGHT] = joints_[RIGHT]->GetVelocity(0) * (wheel_diameter_ / 2.0);

            if (wheel_accel_ == 0 ||
                (fabs(wheel_speed_[LEFT] - current_speed[LEFT]) < 0.01) ||
                (fabs(wheel_speed_[RIGHT] - current_speed[RIGHT]) < 0.01)) {
                //if max_accel == 0, or target speed is reached
                joints_[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT] / (wheel_diameter_ / 2.0));
                joints_[LEFT_BACK]->SetParam("vel", 0, wheel_speed_[LEFT] / (wheel_diameter_ / 2.0));
                joints_[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT] / (wheel_diameter_ / 2.0));
                joints_[RIGHT_BACK]->SetParam("vel", 0, wheel_speed_[RIGHT] / (wheel_diameter_ / 2.0));
            } else {
                if (wheel_speed_[LEFT] >= current_speed[LEFT])
                    wheel_speed_instr_[LEFT] += fmin(wheel_speed_[LEFT] - current_speed[LEFT],
                                                     wheel_accel_ * seconds_since_last_update);
                else
                    wheel_speed_instr_[LEFT] += fmax(wheel_speed_[LEFT] - current_speed[LEFT],
                                                     -wheel_accel_ * seconds_since_last_update);

                if (wheel_speed_[RIGHT] > current_speed[RIGHT])
                    wheel_speed_instr_[RIGHT] += fmin(wheel_speed_[RIGHT] - current_speed[RIGHT],
                                                      wheel_accel_ * seconds_since_last_update);
                else
                    wheel_speed_instr_[RIGHT] += fmax(wheel_speed_[RIGHT] - current_speed[RIGHT],
                                                      -wheel_accel_ * seconds_since_last_update);

                // ROS_INFO_NAMED("four_wheel_diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
                // ROS_INFO_NAMED("four_wheel_diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);

                joints_[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
                joints_[LEFT_BACK]->SetParam("vel", 0, wheel_speed_instr_[LEFT] / (wheel_diameter_ / 2.0));
                joints_[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
                joints_[RIGHT_BACK]->SetParam("vel", 0, wheel_speed_instr_[RIGHT] / (wheel_diameter_ / 2.0));
            }
            last_update_time_ += common::Time(update_period_);
        }
    }

// Finalize the controller
    void FourWheelDiffDriver::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void FourWheelDiffDriver::getWheelVelocities() {
        boost::mutex::scoped_lock scoped_lock(lock_);

        double vr = x_;
        double va = rot_;

        if (legacy_mode_) {
            wheel_speed_[LEFT] = vr + va * wheel_separation_ / 2.0;
            wheel_speed_[RIGHT] = vr - va * wheel_separation_ / 2.0;
        } else {
            wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
            wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
        }
    }

    void FourWheelDiffDriver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg) {
        boost::mutex::scoped_lock scoped_lock(lock_);
        x_ = cmd_msg->linear.x;
        rot_ = cmd_msg->angular.z;
    }

    void FourWheelDiffDriver::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void FourWheelDiffDriver::UpdateOdometryEncoder() {
        double vl_f = joints_[LEFT_FRONT]->GetVelocity(0);
        double vl_b = joints_[LEFT_BACK]->GetVelocity(0);
        double vr_f = joints_[RIGHT_FRONT]->GetVelocity(0);
        double vr_b = joints_[RIGHT_BACK]->GetVelocity(0);

        double vl = (vl_f + vl_b) / 2.0;
        double vr = (vr_f + vr_b) / 2.0;
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = parent_->GetWorld()->SimTime();
#else
        common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
        double seconds_since_last_update = (current_time - last_odom_update_).Double();
        last_odom_update_ = current_time;

        double b = wheel_separation_;

        // Book: Sigwart 2011 Autonompus Mobile Robots page:337
        double sl = vl * (wheel_diameter_ / 2.0) * seconds_since_last_update;
        double sr = vr * (wheel_diameter_ / 2.0) * seconds_since_last_update;
        double ssum = sl + sr;

        double sdiff;
        if (legacy_mode_) {
            sdiff = sl - sr;
        } else {

            sdiff = sr - sl;
        }

        double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
        double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
        double dtheta = (sdiff) / b;

        pose_encoder_.x += dx;
        pose_encoder_.y += dy;
        pose_encoder_.theta += dtheta;

        double w = dtheta / seconds_since_last_update;
        double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

        tf::Quaternion qt;
        tf::Vector3 vt;
        qt.setRPY(0, 0, pose_encoder_.theta);
        vt = tf::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        odom_.twist.twist.angular.z = w;
        odom_.twist.twist.linear.x = dx / seconds_since_last_update;
        odom_.twist.twist.linear.y = dy / seconds_since_last_update;
    }

    void FourWheelDiffDriver::publishOdomTF() {
        tf::Quaternion qt = tf::Quaternion(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
                                           odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
        tf::Vector3 vt = tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                                     odom_.pose.pose.position.z);
        tf::Transform base_footprint_to_odom(qt, vt);
        transform_broadcaster_->sendTransform(
                tf::StampedTransform(base_footprint_to_odom, ros::Time::now(),
                                     odometry_frame_, robot_base_frame_));
    }

    void FourWheelDiffDriver::publishWorldTF() {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent_->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        tf::Quaternion qt = tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::Vector3 vt = tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        // convert velocity to child_frame_id (aka base_footprint)
        tf::Transform base_footprint_to_world(qt, vt);
        transform_broadcaster_->sendTransform(
                tf::StampedTransform(base_footprint_to_world.inverse(), ros::Time::now(),
                                     robot_base_frame_, world_frame_));
    }

    void FourWheelDiffDriver::publishOdometry() {
        // set covariance
        odom_.pose.covariance[0] = 0.00001;
        odom_.pose.covariance[7] = 0.00001;
        odom_.pose.covariance[14] = 1000000000000.0;
        odom_.pose.covariance[21] = 1000000000000.0;
        odom_.pose.covariance[28] = 1000000000000.0;
        odom_.pose.covariance[35] = 0.001;

        // set header
        odom_.header.stamp = ros::Time::now();
        odom_.header.frame_id = odometry_frame_;
        odom_.child_frame_id = robot_base_frame_;

        odometry_publisher_.publish(odom_);
    }

    GZ_REGISTER_MODEL_PLUGIN (FourWheelDiffDriver)
}