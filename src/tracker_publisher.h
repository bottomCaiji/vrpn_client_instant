/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#ifndef SRC_TRACKER_PUBLISHER_H
#define SRC_TRACKER_PUBLISHER_H

#include "vrpn_client_instant/vrpn_client_instant.h"
#include "ros/callback_queue.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

#include <boost/make_shared.hpp>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace vrpn_client_instant
{
  class PublishCallback : public ros::CallbackInterface
  {
  public:
    explicit PublishCallback(std::function<void()> callback) : callback_(std::move(callback)) {}

    CallResult call() override
    {
      callback_();
      return Success;
    }

  private:
    std::function<void()> callback_;
  };

  // Network callbacks only copy reports into the NodeHandle's ROS callback
  // queue. Publication state is kept alive by each queued callback, and guarded
  // even when the application services that queue with a multi-threaded spinner.
  class TrackerPublisher : public std::enable_shared_from_this<TrackerPublisher>
  {
  public:
    TrackerPublisher(const std::string &name, ros::NodeHandle nh)
      : output_nh_(nh, name), callback_queue_(nh.getCallbackQueue()), tracker_name_(name),
        active_(true), scheduled_(false)
    {
      nh.param<std::string>("frame_id", frame_id_, "world");
      nh.param<bool>("use_server_time", use_server_time_, false);
      nh.param<bool>("broadcast_tf", broadcast_tf_, false);
      nh.param<bool>("process_sensor_id", process_sensor_id_, false);
    }

    template <typename Report>
    void enqueue(const Report &report)
    {
      // Capture receive time here, before any ROS callback queue delay.
      const ros::Time stamp = use_server_time_
          ? ros::Time(report.msg_time.tv_sec, report.msg_time.tv_usec * 1000)
          : ros::Time::now();
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!active_.load())
      {
        return;
      }
      pending_.emplace_back([report, stamp](TrackerPublisher &self)
      {
        self.publish(report, stamp);
      });  // NOLINT(whitespace/braces): lambda argument.
      if (!scheduled_)
      {
        schedule();
      }
    }

    void stop()
    {
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        active_.store(false);
        pending_.clear();
      }
      // Already-running callbacks hold their own shared_ptr and see active_=false.
      callback_queue_->removeByID(reinterpret_cast<uint64_t>(this));
      // Wait for an in-flight publication, without holding the ingress lock.
      std::lock_guard<std::mutex> lock(publish_mutex_);
    }

  private:
    // Called with queue_mutex_ held. At most one dispatch is queued/running, so
    // report order is preserved with any number of ROS spinner threads.
    void schedule()
    {
      scheduled_ = true;
      const auto self = shared_from_this();
      callback_queue_->addCallback(boost::make_shared<PublishCallback>([self]() { self->dispatch(); }),
                                   reinterpret_cast<uint64_t>(this));
    }

    void dispatch()
    {
      std::function<void(TrackerPublisher &)> report;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!active_.load() || pending_.empty())
        {
          scheduled_ = false;
          return;
        }
        report = std::move(pending_.front());
        pending_.pop_front();
      }
      {
        std::lock_guard<std::mutex> lock(publish_mutex_);
        if (active_.load())
        {
          report(*this);
        }
      }
      std::lock_guard<std::mutex> lock(queue_mutex_);
      scheduled_ = false;
      if (active_.load() && !pending_.empty())
      {
        schedule();
      }
    }

    template <typename Message>
    ros::Publisher &publisher(std::map<int, ros::Publisher> &publishers, const char *topic, int sensor)
    {
      const int key = process_sensor_id_ ? sensor : 0;
      ros::Publisher &pub = publishers[key];
      if (pub.getTopic().empty())
      {
        ros::NodeHandle nh = process_sensor_id_
            ? ros::NodeHandle(output_nh_, std::to_string(sensor)) : output_nh_;
        pub = nh.advertise<Message>(topic, 1);
      }
      return pub;
    }

    bool validSensor(int sensor) const
    {
      if (process_sensor_id_ && sensor < 0)
      {
        ROS_WARN_STREAM("Ignoring negative VRPN sensor ID " << sensor << " for " << tracker_name_);
        return false;
      }
      return true;
    }

    void publish(const vrpn_TRACKERCB &report, const ros::Time &stamp)
    {
      if (!validSensor(report.sensor))
      {
        return;
      }
      geometry_msgs::PoseStamped message;
      message.header.frame_id = frame_id_;
      message.header.stamp = stamp;
      message.pose.position.x = report.pos[0];
      message.pose.position.y = report.pos[1];
      message.pose.position.z = report.pos[2];
      message.pose.orientation.x = report.quat[0];
      message.pose.orientation.y = report.quat[1];
      message.pose.orientation.z = report.quat[2];
      message.pose.orientation.w = report.quat[3];
      publisher<geometry_msgs::PoseStamped>(pose_pubs_, "pose", report.sensor).publish(message);

      if (broadcast_tf_)
      {
        if (!tf_broadcaster_)
        {
          tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
        }
        geometry_msgs::TransformStamped transform;
        transform.header = message.header;
        transform.child_frame_id = process_sensor_id_
            ? tracker_name_ + "/" + std::to_string(report.sensor) : tracker_name_;
        transform.transform.translation.x = report.pos[0];
        transform.transform.translation.y = report.pos[1];
        transform.transform.translation.z = report.pos[2];
        transform.transform.rotation = message.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
      }
    }

    void publish(const vrpn_TRACKERVELCB &report, const ros::Time &stamp)
    {
      if (!validSensor(report.sensor))
      {
        return;
      }
      geometry_msgs::TwistStamped message;
      message.header.frame_id = frame_id_;
      message.header.stamp = stamp;
      message.twist.linear.x = report.vel[0];
      message.twist.linear.y = report.vel[1];
      message.twist.linear.z = report.vel[2];
      tf2::Matrix3x3(tf2::Quaternion(report.vel_quat[0], report.vel_quat[1],
                                   report.vel_quat[2], report.vel_quat[3]))
          .getRPY(message.twist.angular.x, message.twist.angular.y, message.twist.angular.z);
      publisher<geometry_msgs::TwistStamped>(twist_pubs_, "twist", report.sensor).publish(message);
    }

    void publish(const vrpn_TRACKERACCCB &report, const ros::Time &stamp)
    {
      if (!validSensor(report.sensor))
      {
        return;
      }
      geometry_msgs::AccelStamped message;
      message.header.frame_id = frame_id_;
      message.header.stamp = stamp;
      message.accel.linear.x = report.acc[0];
      message.accel.linear.y = report.acc[1];
      message.accel.linear.z = report.acc[2];
      tf2::Matrix3x3(tf2::Quaternion(report.acc_quat[0], report.acc_quat[1],
                                   report.acc_quat[2], report.acc_quat[3]))
          .getRPY(message.accel.angular.x, message.accel.angular.y, message.accel.angular.z);
      publisher<geometry_msgs::AccelStamped>(accel_pubs_, "accel", report.sensor).publish(message);
    }

    ros::NodeHandle output_nh_;
    ros::CallbackQueueInterface *callback_queue_;
    std::string tracker_name_, frame_id_;
    std::atomic<bool> active_;
    bool scheduled_, use_server_time_, broadcast_tf_, process_sensor_id_;
    std::mutex queue_mutex_, publish_mutex_;
    std::deque<std::function<void(TrackerPublisher &)>> pending_;
    std::map<int, ros::Publisher> pose_pubs_, twist_pubs_, accel_pubs_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  };

}  // namespace vrpn_client_instant

#endif  // SRC_TRACKER_PUBLISHER_H
