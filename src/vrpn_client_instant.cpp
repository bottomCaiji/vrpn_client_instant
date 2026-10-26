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

#include "vrpn_client_instant/vrpn_client_instant.h"

#include "tracker_publisher.h"  // NOLINT(build/include): private header beside this source file.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace
{
  const std::unordered_set<std::string> name_blacklist_({"VRPN Control"});

  // VRPN owns a reference for every get_connection_by_name() call. Release it
  // through VRPN rather than deleting a possibly shared connection directly.
  vrpn_client_instant::ConnectionPtr connect(const std::string &host)
  {
    // Each receiver thread must own a separate connection, even when multiple
    // clients in one process use the same host. Trackers explicitly share ours.
    vrpn_Connection *connection = vrpn_get_connection_by_name(host.c_str(), NULL, NULL, NULL, NULL, NULL, true);
    if (!connection)
    {
      throw std::runtime_error("Unable to create VRPN connection to " + host);
    }
    return vrpn_client_instant::ConnectionPtr(connection, [](vrpn_Connection *value)
    {
      value->removeReference();
    });  // NOLINT(whitespace/braces): lambda argument.
  }

  bool isInvalidFirstCharInName(const char c)
  {
    return !(std::isalpha(static_cast<unsigned char>(c)) || c == '/' || c == '~');
  }

  bool isInvalidSubsequentCharInName(const char c)
  {
    return !(std::isalnum(static_cast<unsigned char>(c)) || c == '/' || c == '_');
  }
}  // namespace

namespace vrpn_client_instant
{
  // The timeout only bounds shutdown/maintenance latency. select() inside VRPN
  // wakes immediately on either TCP or UDP input; it does not rate-limit reports.
  class VrpnMainloop
  {
  public:
    explicit VrpnMainloop(std::function<bool()> receive)
      : stopping_(false), thread_([this, receive]()
      {
        while (!stopping_.load() && ros::ok())
        {
          const bool connected = receive();
          if (!connected)
          {
            // VRPN's reconnect path may return without waiting on a socket.
            std::unique_lock<std::mutex> lock(wait_mutex_);
            wakeup_.wait_for(lock, std::chrono::milliseconds(10), [this]() { return stopping_.load(); });
          }
        }
      })  // NOLINT(whitespace/braces): lambda in initializer list.
    {
    }

    ~VrpnMainloop()
    {
      {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        stopping_.store(true);
      }
      wakeup_.notify_all();
      thread_.join();
    }

  private:
    std::atomic<bool> stopping_;
    std::mutex wait_mutex_;
    std::condition_variable wakeup_;
    std::thread thread_;
  };

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh)
    : connection_(connection)
  {
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection_.get());
    std::string clean_name = tracker_name;
    std::size_t start_subsequent = 1;
    if (!clean_name.empty() && isInvalidFirstCharInName(clean_name[0]))
    {
      clean_name.erase(0, 1);
      start_subsequent = 0;
    }
    if (!clean_name.empty())
    {
      clean_name.erase(std::remove_if(clean_name.begin() + start_subsequent, clean_name.end(),
                                     isInvalidSubsequentCharInName), clean_name.end());
    }
    init(clean_name, nh);
  }

  VrpnTrackerRos::VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh)
    : connection_(connect(host))
  {
    tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(tracker_name.c_str(), connection_.get());
    init(tracker_name, nh);
    mainloop_.reset(new VrpnMainloop([this]() { return receive(); }));
  }

  void VrpnTrackerRos::init(std::string tracker_name, ros::NodeHandle nh)
  {
    std::string error;
    if (tracker_name.empty() || !ros::names::validate(tracker_name, error))
    {
      throw std::invalid_argument("Invalid tracker name " + tracker_name + ": " + error);
    }
    ROS_INFO_STREAM("Creating new tracker " << tracker_name);
    publisher_ = std::make_shared<TrackerPublisher>(tracker_name, nh);
    tracker_remote_->shutup = true;
    // Register only after all callback state is ready.
    tracker_remote_->register_change_handler(publisher_.get(), &VrpnTrackerRos::handle_pose);
    tracker_remote_->register_change_handler(publisher_.get(), &VrpnTrackerRos::handle_twist);
    tracker_remote_->register_change_handler(publisher_.get(), &VrpnTrackerRos::handle_accel);
    if (nh.hasParam("update_frequency"))
    {
      ROS_WARN_ONCE("update_frequency is ignored; VRPN reports are now processed on arrival");
    }
  }

  VrpnTrackerRos::~VrpnTrackerRos()
  {
    // Stop/join producers before unregistering handlers or releasing ROS state.
    mainloop_.reset();
    std::lock_guard<std::mutex> lock(vrpn_mutex_);
    tracker_remote_->unregister_change_handler(publisher_.get(), &VrpnTrackerRos::handle_pose);
    tracker_remote_->unregister_change_handler(publisher_.get(), &VrpnTrackerRos::handle_twist);
    tracker_remote_->unregister_change_handler(publisher_.get(), &VrpnTrackerRos::handle_accel);
    tracker_remote_.reset();
    publisher_->stop();
  }

  void VrpnTrackerRos::mainloop()
  {
    std::lock_guard<std::mutex> lock(vrpn_mutex_);
    tracker_remote_->mainloop();
  }

  bool VrpnTrackerRos::receive()
  {
    std::lock_guard<std::mutex> lock(vrpn_mutex_);
    const timeval timeout = {0, 100000};
    connection_->mainloop(&timeout);
    tracker_remote_->mainloop();
    return connection_->connected();
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_pose(void *userData, const vrpn_TRACKERCB report)
  {
    static_cast<TrackerPublisher *>(userData)->enqueue(report);
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_twist(void *userData, const vrpn_TRACKERVELCB report)
  {
    static_cast<TrackerPublisher *>(userData)->enqueue(report);
  }

  void VRPN_CALLBACK VrpnTrackerRos::handle_accel(void *userData, const vrpn_TRACKERACCCB report)
  {
    static_cast<TrackerPublisher *>(userData)->enqueue(report);
  }

  VrpnClientRos::VrpnClientRos(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : output_nh_(private_nh), refresh_tracker_frequency_(0.0),
      next_tracker_refresh_(std::chrono::steady_clock::now())
  {
    host_ = getHostStringFromParams(private_nh);
    ROS_INFO_STREAM("Connecting to VRPN server at " << host_);
    connection_ = connect(host_);
    private_nh.param<double>("refresh_tracker_frequency", refresh_tracker_frequency_, 0.0);
    if (!std::isfinite(refresh_tracker_frequency_))
    {
      throw std::invalid_argument("refresh_tracker_frequency must be finite");
    }
    std::vector<std::string> names;
    if (private_nh.getParam("trackers", names))
    {
      for (const auto &name : names)
      {
        trackers_.emplace(name, std::make_shared<VrpnTrackerRos>(name, connection_, output_nh_));
      }
    }
    // Start only after all trackers and shared connection state are initialized.
    mainloop_.reset(new VrpnMainloop([this]() { return receive(); }));
  }

  VrpnClientRos::~VrpnClientRos()
  {
    mainloop_.reset();
    trackers_.clear();
  }

  std::string VrpnClientRos::getHostStringFromParams(ros::NodeHandle host_nh)
  {
    std::stringstream host_stream;
    std::string server;
    int port;
    host_nh.param<std::string>("server", server, "localhost");
    host_stream << server;
    if (host_nh.getParam("port", port))
    {
      host_stream << ":" << port;
    }
    return host_stream.str();
  }

  void VrpnClientRos::mainloop()
  {
    receive();
  }

  bool VrpnClientRos::receive()
  {
    std::lock_guard<std::mutex> lock(vrpn_mutex_);
    const timeval timeout = {0, 100000};
    connection_->mainloop(&timeout);
    if (!connection_->doing_okay())
    {
      ROS_WARN_THROTTLE(5.0, "VRPN connection is not 'doing okay'");
    }
    const auto now = std::chrono::steady_clock::now();
    if (refresh_tracker_frequency_ > 0.0 && now >= next_tracker_refresh_)
    {
      updateTrackersLocked();
      next_tracker_refresh_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / refresh_tracker_frequency_));
    }
    for (const auto &tracker : trackers_)
    {
      tracker.second->mainloop();
    }
    return connection_->connected();
  }

  void VrpnClientRos::updateTrackers()
  {
    std::lock_guard<std::mutex> lock(vrpn_mutex_);
    updateTrackersLocked();
  }

  void VrpnClientRos::updateTrackersLocked()
  {
    for (int i = 0; connection_->sender_name(i) != NULL; ++i)
    {
      const std::string name(connection_->sender_name(i));
      if (trackers_.count(name) == 0 && name_blacklist_.count(name) == 0)
      {
        ROS_INFO_STREAM("Found new sender: " << name);
        try
        {
          trackers_.emplace(name, std::make_shared<VrpnTrackerRos>(name, connection_, output_nh_));
        }
        catch (const std::invalid_argument &error)
        {
          ROS_WARN_STREAM_THROTTLE(5.0, "Skipping VRPN sender: " << error.what());
        }
      }
    }
  }
}  // namespace vrpn_client_instant
