// Copyright 2026 vrpn_client_instant contributors.
// SPDX-License-Identifier: BSD-3-Clause

#include "../src/tracker_publisher.h"

#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
  vrpn_TRACKERCB pose(int index, int sensor = 0)
  {
    vrpn_TRACKERCB report = {};
    report.sensor = sensor;
    report.msg_time.tv_sec = 42;
    report.msg_time.tv_usec = index < 0 ? 0 : index;
    report.pos[0] = index;
    report.pos[1] = sensor;
    report.pos[2] = -index;
    report.quat[3] = 1.0;
    return report;
  }

  class TrackerPublisherTest : public testing::Test
  {
  protected:
    TrackerPublisherTest()
      : nh_(std::string("~") + testing::UnitTest::GetInstance()->current_test_info()->name()),
        subscriber_spinner_(1)
    {
      nh_.setCallbackQueue(&publication_queue_);
    }

    void SetUp() override
    {
      nh_.setParam("use_server_time", useServerTime());
      publisher_ = std::make_shared<vrpn_client_instant::TrackerPublisher>("tracker", nh_);
      ros::NodeHandle subscriber_nh;
      pose_sub_ = subscriber_nh.subscribe<geometry_msgs::PoseStamped>(nh_.resolveName("tracker/pose"), 10000,
          [this](const geometry_msgs::PoseStamped::ConstPtr &message)
          {
            if (message->pose.position.x < 0)  // Advertisement/warm-up report.
            {
              return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            received_.push_back(*message);
            received_condition_.notify_all();
          });  // NOLINT(whitespace/braces): lambda argument.
      subscriber_spinner_.start();
      publisher_->enqueue(pose(-1));
      publication_queue_.callAvailable();
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (pose_sub_.getNumPublishers() == 0 && std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      ASSERT_GT(pose_sub_.getNumPublishers(), 0u);
    }

    void TearDown() override
    {
      publisher_->stop();
      subscriber_spinner_.stop();
      publication_queue_.clear();
    }

    virtual bool useServerTime() const { return true; }

    bool waitForReports(std::size_t count)
    {
      std::unique_lock<std::mutex> lock(mutex_);
      return received_condition_.wait_for(lock, std::chrono::seconds(10),
                                          [this, count]() { return received_.size() >= count; });
    }

    ros::CallbackQueue publication_queue_;
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::AsyncSpinner subscriber_spinner_;
    std::shared_ptr<vrpn_client_instant::TrackerPublisher> publisher_;
    std::mutex mutex_;
    std::condition_variable received_condition_;
    std::vector<geometry_msgs::PoseStamped> received_;
  };

  TEST_F(TrackerPublisherTest, BurstPreservesReportsAndServerTime)
  {
    const int count = 2000;
    std::thread network([this]()
    {
      for (int i = 0; i < count; ++i)
      {
        publisher_->enqueue(pose(i));
      }
    });  // NOLINT(whitespace/braces): lambda argument.
    network.join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      EXPECT_TRUE(received_.empty());  // Network callbacks have not published.
    }

    ros::AsyncSpinner spinner(4, &publication_queue_);
    spinner.start();
    ASSERT_TRUE(waitForReports(count));
    spinner.stop();
    std::lock_guard<std::mutex> lock(mutex_);
    ASSERT_EQ(static_cast<std::size_t>(count), received_.size());
    for (int i = 0; i < count; ++i)
    {
      EXPECT_DOUBLE_EQ(i, received_[i].pose.position.x);
      EXPECT_DOUBLE_EQ(-i, received_[i].pose.position.z);
      EXPECT_EQ(ros::Time(42, i * 1000), received_[i].header.stamp);
    }
  }

  TEST_F(TrackerPublisherTest, ConcurrentCallbacksKeepEachProducerInOrder)
  {
    const int producers = 4;
    const int count = 500;
    ros::AsyncSpinner spinner(4, &publication_queue_);
    spinner.start();
    std::vector<std::thread> threads;
    for (int sensor = 0; sensor < producers; ++sensor)
    {
      threads.emplace_back([this, sensor]()
      {
        for (int i = 0; i < count; ++i)
        {
          publisher_->enqueue(pose(i, sensor));
        }
      });  // NOLINT(whitespace/braces): lambda argument.
    }
    for (auto &thread : threads)
    {
      thread.join();
    }
    ASSERT_TRUE(waitForReports(producers * count));
    spinner.stop();
    std::lock_guard<std::mutex> lock(mutex_);
    ASSERT_EQ(static_cast<std::size_t>(producers * count), received_.size());
    std::vector<int> next(producers, 0);
    for (const auto &message : received_)
    {
      const int sensor = static_cast<int>(message.pose.position.y);
      ASSERT_GE(sensor, 0);
      ASSERT_LT(sensor, producers);
      EXPECT_DOUBLE_EQ(next[sensor]++, message.pose.position.x);
      EXPECT_DOUBLE_EQ(-message.pose.position.x, message.pose.position.z);
    }
  }

  TEST_F(TrackerPublisherTest, StopCancelsQueuedAndFutureReports)
  {
    for (int i = 0; i < 100; ++i)
    {
      publisher_->enqueue(pose(i));
    }
    publisher_->stop();
    publisher_->enqueue(pose(101));
    publication_queue_.callAvailable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(received_.empty());
    EXPECT_TRUE(publication_queue_.isEmpty());
  }

  class ClientTimePublisherTest : public TrackerPublisherTest
  {
    bool useServerTime() const override { return false; }
  };

  TEST_F(ClientTimePublisherTest, ClientTimeIsCapturedBeforeQueueWait)
  {
    const ros::Time before = ros::Time::now();
    publisher_->enqueue(pose(0));
    const ros::Time after = ros::Time::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    publication_queue_.callAvailable();
    ASSERT_TRUE(waitForReports(1));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GE(received_[0].header.stamp, before);
    EXPECT_LE(received_[0].header.stamp, after);
  }
}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_tracker_publisher");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
