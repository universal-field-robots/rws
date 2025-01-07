
#ifndef RWS__CONNECTOR_HPP_
#define RWS__CONNECTOR_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rws/node_interface.hpp"

namespace rws
{

struct topic_params
{
  topic_params() : history_depth(10), compression("none"), topic(""), type(""), latch(false), throttle_rate(0, 0) {}
  topic_params(std::string t, std::string tp)
  : history_depth(10), compression("none"), topic(t), type(tp), latch(false), throttle_rate(0, 0)
  {
  }
  topic_params(std::string t, std::string tp, size_t qs, std::string c, rclcpp::Duration tr)
  : history_depth(qs), compression(c), topic(t), type(tp), latch(false), throttle_rate(tr)
  {
  }
  topic_params(std::string t, std::string tp, size_t qs, bool l)
  : history_depth(qs), compression("none"), topic(t), type(tp), latch(l), throttle_rate(0, 0)
  {
  }
  bool operator==(const topic_params & p)
  {
    return  topic == p.topic && 
            type == p.type && 
            history_depth == p.history_depth && 
            throttle_rate == p.throttle_rate &&
            compression == p.compression &&
            latch == p.latch;
  }
  size_t history_depth;
  std::string compression;  // rws internal
  std::string topic;
  std::string type;
  bool latch;  // only for publishers, rws internal
  rclcpp::Duration throttle_rate;
};

typedef std::function<void(topic_params & params, std::shared_ptr<const rclcpp::SerializedMessage> message)>
  subscription_callback;

template <class PublisherClass = rclcpp::GenericPublisher>
class Connector
{
public:
  Connector(std::shared_ptr<NodeInterface<PublisherClass>> node) : node_(node) {}
  ~Connector() {}

  std::function<void()> subscribe_to_topic(
    uint16_t client_id, topic_params & params, subscription_callback handler)
  {
    std::lock_guard<std::mutex> guard(subscribers_mutex_);

    auto matching_subscriber = get_subscriber_by_params(params);
    subscriber_handle handle = {nullptr, params, handler, client_id, next_handler_id_++, rclcpp::Time(0, 0, RCL_ROS_TIME)};
    rclcpp::QoS qos(params.history_depth);
    auto info = node_->get_publishers_info_by_topic(params.topic);
    for(const auto& node : info)
    {
      qos.durability(node.qos_profile().get_rmw_qos_profile().durability);
      qos.reliability(node.qos_profile().get_rmw_qos_profile().reliability);
    }

    bool is_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;

    if (matching_subscriber == nullptr) {
      handle.subscription = node_->create_generic_subscription(
        params.topic, params.type, qos,
        std::bind(&Connector::topic_message_callback, this, params, std::placeholders::_1));
    } else {
      handle.subscription = matching_subscriber->subscription;

      if (is_transient_local) {
        // Make a temporary subscription for the latched topic to ensure the new subscriber gets any existing messages
        std::thread([=]() {
          std::atomic<bool> fired = false;

          auto oneshot_sub = node_->create_generic_subscription(
            params.topic, params.type, qos,
            std::bind(
              [&fired, &handler](topic_params & params, std::shared_ptr<const rclcpp::SerializedMessage> message) {
                handler(params, message);

                fired = true;
              },params, std::placeholders::_1));

          while (!fired) {
            std::this_thread::yield();
          }

          oneshot_sub.reset();
        }).detach();
      }
    }

    subscribers_.push_back(handle);

    return [this, handle_id = handle.handle_id]() {
      std::lock_guard<std::mutex> guard(subscribers_mutex_);
      for (auto it = subscribers_.begin(); it != subscribers_.end(); ++it) {
        if ((*it).handle_id == handle_id) {
          subscribers_.erase(it);
          break;
        }
      }
    };
  }

  bool is_subscribed_to_topic(topic_params & params)
  {
    return get_subscriber_by_params(params) != nullptr;
  }

  bool is_advertising_topic(topic_params & params)
  {
    return get_publisher_by_params(params) != nullptr;
  }

  std::function<void()> advertise_topic(
    uint16_t client_id, topic_params & params,
    std::function<void(std::shared_ptr<const rclcpp::SerializedMessage>)> & cb_in)
  {
    std::lock_guard<std::mutex> guard(publishers_mutex_);

    auto matching_publisher = get_publisher_by_params(params);
    publisher_handle handle({nullptr, params, client_id, next_handler_id_++});

    auto qos = rclcpp::QoS(params.history_depth);
    if (params.latch) {
      qos = qos.transient_local();
    }
    if (matching_publisher == nullptr) {
      handle.publisher = node_->create_generic_publisher(params.topic, params.type, qos);
    } else {
      handle.publisher = matching_publisher->publisher;
    }

    publishers_.push_back(handle);

    cb_in = [p = handle.publisher](std::shared_ptr<const rclcpp::SerializedMessage> message) {
      p->publish(*message);
    };

    return [this, handle_id = handle.handle_id]() {
      std::lock_guard<std::mutex> guard(publishers_mutex_);
      for (auto it = publishers_.begin(); it != publishers_.end(); ++it) {
        if ((*it).handle_id == handle_id) {
          publishers_.erase(it);
          break;
        }
      }
    };
  }

private:
  struct subscriber_handle
  {
    std::shared_ptr<rclcpp::GenericSubscription> subscription;
    topic_params params;
    subscription_callback callback;
    size_t client_id;
    size_t handle_id;
    rclcpp::Time last_sent;
  };
  struct publisher_handle
  {
    std::shared_ptr<PublisherClass> publisher;
    topic_params params;
    size_t client_id;
    size_t handle_id;
  };

  std::shared_ptr<NodeInterface<PublisherClass>> node_;
  std::vector<subscriber_handle> subscribers_;
  std::vector<publisher_handle> publishers_;
  std::mutex subscribers_mutex_;
  std::mutex publishers_mutex_;
  size_t next_handler_id_ = 0;

  subscriber_handle * get_subscriber_by_params(topic_params & params)
  {
    for (auto & sub : subscribers_) {
      if (sub.params == params) {
        return &sub;
      }
    }
    return nullptr;
  }

  publisher_handle * get_publisher_by_params(topic_params & params)
  {
    for (auto & pub : publishers_) {
      if (pub.params == params) {
        return &pub;
      }
    }
    return nullptr;
  }

  void topic_message_callback(topic_params & params, std::shared_ptr<const rclcpp::SerializedMessage> message)
  {
    std::lock_guard<std::mutex> guard(subscribers_mutex_);
    for (auto & sub : subscribers_) {
      if (sub.params == params &&
          (params.throttle_rate.nanoseconds() == 0 || (sub.last_sent + params.throttle_rate) < node_->now())) {
        sub.callback(params, message);
        sub.last_sent = node_->now();
      }
    }
  }
};

}  // namespace rws

#endif  // RWS__CONNECTOR_HPP_
