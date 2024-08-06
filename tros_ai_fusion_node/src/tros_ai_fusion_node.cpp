#include "tros_ai_fusion_node.h"

namespace tros {

TrosAiMsgFusionNode::TrosAiMsgFusionNode(const rclcpp::NodeOptions &options) :
  Node("tros_ai_fusion", options) {
  RCLCPP_INFO(this->get_logger(), "TrosAiMsgFusionNode is initializing...");
  this->declare_parameter("topic_names_fusion", rclcpp::ParameterValue(fusion_topic_names_));
  this->get_parameter("topic_names_fusion", fusion_topic_names_);
  fusion_topic_name_base_ = this->declare_parameter("topic_name_base", fusion_topic_name_base_);
  pub_fusion_topic_name_= this->declare_parameter("pub_fusion_topic_name", pub_fusion_topic_name_);

  // timer_callback();
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
  //   std::bind(&TrosAiMsgFusionNode::timer_callback, this));

  std::stringstream ss;
  for (const auto &topic_name : fusion_topic_names_) {
    ss << "\n\t" << topic_name;
  }
  RCLCPP_WARN_STREAM(this->get_logger(),
    "\n topic_name_base [" << fusion_topic_name_base_ << "]"
    << "\n topic_names_fusion: " << ss.str()
    << "\n pub_fusion_topic_name [" << pub_fusion_topic_name_ << "]"
  );

  ai_msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
    pub_fusion_topic_name_,
    rclcpp::QoS(10));
  base_sub_.subscribe(this, fusion_topic_name_base_);
  srv_topic_manage_ = this->create_service<tros_ai_fusion_msgs::srv::TopicManage>(
    "topic_manage",
    std::bind(&TrosAiMsgFusionNode::topic_manage_callback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
  for (auto & topic : fusion_topic_names_) {
    if (topic.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "topic name is empty");
      continue;
    } else {
      if (synchronizers_map_.find(topic) != synchronizers_map_.end()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
          "topic name already exists: " << topic);
        continue;
      } else {
        RCLCPP_WARN_STREAM(this->get_logger(),
          "topic: " << topic << ", sync with base topic: " << fusion_topic_name_base_);
        subs_map_[topic].subscribe(this, topic);
        auto sync = std::make_shared<SynchronizerType>(CustomSyncPolicyType(10), base_sub_, subs_map_[topic]);
        synchronizers_map_[topic] = sync;
        synchronizers_map_[topic]->registerCallback(std::bind(&TrosAiMsgFusionNode::TopicSyncCallback,
          this, fusion_topic_name_base_, topic,
          std::placeholders::_1, std::placeholders::_2));
      }
    }
  }

  RCLCPP_WARN_STREAM(get_logger(),
    "fusion_topic_names size:" << fusion_topic_names_.size()
    << " synchronizers_map_ size:" << synchronizers_map_.size());
}

void TrosAiMsgFusionNode::timer_callback() {
}

void TrosAiMsgFusionNode::topic_manage_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage::Request> request,
  const std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage::Response> response) {

  if (request->action == tros_ai_fusion_msgs::srv::TopicManage::Request::ADD) {
    // if (request->topics.empty()) {
    //   response->result = false;
    //   RCLCPP_ERROR_STREAM(this->get_logger(),
    //     "topic name is empty");
    //   return;
    // }

    response->result = true;
    
    for (auto & topic : request->topics) 
    {
      if (topic.empty()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
          "topic name is empty");
        continue;
      } else {
        if (synchronizers_map_.find(topic) != synchronizers_map_.end()) {
          RCLCPP_ERROR_STREAM(this->get_logger(),
            "topic name already exists: " << topic);
          continue;
        } else {
          RCLCPP_WARN_STREAM(this->get_logger(),
            "add topic: " << topic << ", sync with base topic: " << fusion_topic_name_base_);
          subs_map_[topic].subscribe(this, topic);

          auto sync = std::make_shared<SynchronizerType>(CustomSyncPolicyType(10), base_sub_, subs_map_[topic]);
          sync->registerCallback(std::bind(&TrosAiMsgFusionNode::TopicSyncCallback,
            this, fusion_topic_name_base_, topic,
            std::placeholders::_1, std::placeholders::_2));
          synchronizers_map_[topic] = sync;
        }
      }
    }
  } else if (request->action == tros_ai_fusion_msgs::srv::TopicManage::Request::DELETE)
  {
    response->result = true;
    
  } else if (request->action == tros_ai_fusion_msgs::srv::TopicManage::Request::GET)
  {
    response->result = true;
    
  } else {
    RCLCPP_WARN(this->get_logger(),
      "action [%s] is invalid", request->action.c_str());
    response->result = false;
    return;
  }

  return;
}

void TrosAiMsgFusionNode::TopicSyncCallback(
  std::string base_topic, std::string fusion_topic,
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg1,
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg2) {
  // return;

  auto copy_msg = [this](const ai_msgs::msg::PerceptionTargets::ConstSharedPtr in_msg)->ai_msgs::msg::PerceptionTargets::SharedPtr{
    auto msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();
    msg->set__header(in_msg->header);
    msg->set__fps(in_msg->fps);
    msg->set__perfs(in_msg->perfs);
    msg->set__targets(in_msg->targets);
    msg->set__disappeared_targets(in_msg->disappeared_targets);

    return msg;
  };

  std::string time = std::to_string(msg1->header.stamp.sec) + "." + std::to_string(msg1->header.stamp.nanosec);

  RCLCPP_WARN_THROTTLE(this->get_logger(),
    *this->get_clock(), 1000,
    "TopicSyncCallback at [%s] with topics [%s] [%s]",
    time.data(), base_topic.data(), fusion_topic.data());

  std::unique_lock<std::mutex> lock(sync_msgs_cache_mutex_);
  auto iter = msg_cache_.find(time);
  if (iter == msg_cache_.end()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Insert msgs at [%s].",
      time.data());
    MsgCacheType msg_cache;
    msg_cache[base_topic] = copy_msg(msg1);
    msg_cache[fusion_topic] = copy_msg(msg2);
    msg_cache_[time] = std::move(msg_cache);
  } else {
    iter->second[fusion_topic] = copy_msg(msg2);
    RCLCPP_INFO(
      this->get_logger(),
      "Messages at [%s] have [%d] msgs.",
      time.data(), iter->second.size());
  }
  
  iter = msg_cache_.find(time);
  if (iter->second.size() == fusion_topic_names_.size() + 1) {
    RCLCPP_INFO(
      this->get_logger(),
      "Messages at [%s] are ready with [%d] msgs.",
      time.data(), iter->second.size());

    FusionMsg(iter->second);
    msg_cache_.erase(iter);

    // MsgCacheType msg_cache = iter->second;
    // FusionMsg(std::move(msg_cache));
    // msg_cache_.erase(iter);
    // std::async(std::launch::async, [this, &msg_cache]() {
      // FusionMsg(std::move(msg_cache));
    // });
  }

  while (msg_cache_.size() > sync_msgs_cache_max_size_) {
    std::stringstream ss;
    ss << "msg_cache_ size:" << msg_cache_.size()
    << " exceeds limit:" << sync_msgs_cache_max_size_
    << ", erase ts:" << msg_cache_.begin()->first
    << ", which has recved " << msg_cache_.begin()->second.size() << " topics [";
    for (auto itr = msg_cache_.begin()->second.begin(); itr != msg_cache_.begin()->second.end(); itr++) {
      ss << itr->first << " ";
    }
    ss << "], should have " << fusion_topic_names_.size() + 1 << " topics [";  
    for (const auto& name : fusion_topic_names_) {
      ss << name << " ";
    }
    ss << fusion_topic_name_base_ << "]";

    RCLCPP_WARN_STREAM(this->get_logger(),
      ss.str()
      );
    
    msg_cache_.erase(msg_cache_.begin());
  }
}


void TrosAiMsgFusionNode::FusionMsg(MsgCacheType msg_cache) {
  assert(ai_msg_publisher_);

  auto pub_ai_msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();
  auto base_msg = msg_cache[fusion_topic_name_base_];

  pub_ai_msg->set__header(base_msg->header);
  pub_ai_msg->set__fps(base_msg->fps);
  pub_ai_msg->set__perfs(base_msg->perfs);

  pub_ai_msg->set__targets(base_msg->targets);
  pub_ai_msg->set__disappeared_targets(base_msg->disappeared_targets);


  RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),
    *this->get_clock(), 1000,
    "Fusion msg at " << base_msg->header.stamp.sec << "." << base_msg->header.stamp.nanosec
    << " with msg size " << msg_cache.size()
  );

  for (auto iter = msg_cache.begin(); iter != msg_cache.end(); ++iter) {
    if (iter->first == fusion_topic_name_base_) {
      continue;
    }
    for (const auto& target : iter->second->targets) {
      pub_ai_msg->targets.push_back(target);
    }
    for (const auto& target : iter->second->disappeared_targets) {
      pub_ai_msg->disappeared_targets.push_back(target);
    }
  }
  
  ai_msg_publisher_->publish(std::move(*pub_ai_msg));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tros::TrosAiMsgFusionNode)