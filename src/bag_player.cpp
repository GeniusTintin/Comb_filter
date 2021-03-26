#include "comb/bag_player.h"

#include <iostream>

#include <rosbag/view.h>

namespace rpg_common_ros {

BagPlayer::BagPlayer(const std::string& file_name)
{
  bag_.open(file_name, rosbag::bagmode::Read);
}

  void BagPlayer::attachCallbackToTopic(const std::string& topic, const std::function<void(const rosbag::MessageInstance&)>& callback){
    VLOG(1) << "Inside function attachCallBackToTopic.";
    CHECK(subscriptions_.emplace(topic, callback).second) << "emplace Failed!";
}

void BagPlayer::play()
{
  VLOG(1) << "Inside function Play.";
  std::vector<std::string> topics;
  for (const CallBackMap::value_type& topic_data : subscriptions_)
  {
    topics.emplace_back(topic_data.first);
  }

  rosbag::View view(bag_, rosbag::TopicQuery(topics));
  for (const rosbag::MessageInstance& message : view)
  {
    const std::string& topic = message.getTopic();
    subscriptions_[topic](message);
  }
}

}  // namespace rpg_common_ros