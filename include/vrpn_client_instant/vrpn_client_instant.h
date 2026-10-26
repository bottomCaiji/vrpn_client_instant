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

#ifndef VRPN_CLIENT_INSTANT_VRPN_CLIENT_INSTANT_H
#define VRPN_CLIENT_INSTANT_VRPN_CLIENT_INSTANT_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace vrpn_client_instant
{

  typedef std::shared_ptr<vrpn_Connection> ConnectionPtr;
  typedef std::shared_ptr<vrpn_Tracker_Remote> TrackerRemotePtr;

  class TrackerPublisher;
  class VrpnMainloop;

  class VrpnTrackerRos
  {
  public:
    typedef std::shared_ptr<VrpnTrackerRos> Ptr;
    /**
     * Create and initialize VrpnTrackerRos using an existing underlying VRPN connection object. The underlying
     * connection owner is responsible for calling mainloop and serializing all VRPN access.
     */
    VrpnTrackerRos(std::string tracker_name, ConnectionPtr connection, ros::NodeHandle nh);

    /**
     * Create and initialize VrpnTrackerRos, creating a new connection to tracker_name@host. This constructor will
     * run a receiving thread which wakes when network data arrives.
     */
    VrpnTrackerRos(std::string tracker_name, std::string host, ros::NodeHandle nh);

    ~VrpnTrackerRos();

    /**
     * Call mainloop of underlying vrpn_Tracker_Remote
     */
    void mainloop();

  private:
    ConnectionPtr connection_;
    TrackerRemotePtr tracker_remote_;
    std::shared_ptr<TrackerPublisher> publisher_;
    std::mutex vrpn_mutex_;
    std::unique_ptr<VrpnMainloop> mainloop_;

    void init(std::string tracker_name, ros::NodeHandle nh);
    bool receive();

    static void VRPN_CALLBACK handle_pose(void *userData, const vrpn_TRACKERCB tracker_pose);

    static void VRPN_CALLBACK handle_twist(void *userData, const vrpn_TRACKERVELCB tracker_twist);

    static void VRPN_CALLBACK handle_accel(void *userData, const vrpn_TRACKERACCCB tracker_accel);
  };

  class VrpnClientRos
  {
  public:
    typedef std::shared_ptr<VrpnClientRos> Ptr;
    typedef std::unordered_map<std::string, VrpnTrackerRos::Ptr> TrackerMap;

    /**
     * Create and initialize VrpnClientRos object in the private_nh namespace.
     */
    VrpnClientRos(ros::NodeHandle nh, ros::NodeHandle private_nh);

    ~VrpnClientRos();

    static std::string getHostStringFromParams(ros::NodeHandle host_nh);

    /**
     * Call mainloop of underlying VRPN connection and all registered VrpnTrackerRemote objects.
     */
    void mainloop();

    /**
     * Examine vrpn_Connection's senders and create new trackers as necessary.
     */
    void updateTrackers();

  private:
    std::string host_;
    ros::NodeHandle output_nh_;

    /**
     * Underlying VRPN connection object
     */
    ConnectionPtr connection_;

    /**
     * Map of registered trackers, accessible by name
     */
    TrackerMap trackers_;

    std::mutex vrpn_mutex_;
    double refresh_tracker_frequency_;
    std::chrono::steady_clock::time_point next_tracker_refresh_;
    std::unique_ptr<VrpnMainloop> mainloop_;

    bool receive();
    void updateTrackersLocked();
  };
}  // namespace vrpn_client_instant

#endif  // VRPN_CLIENT_INSTANT_VRPN_CLIENT_INSTANT_H
