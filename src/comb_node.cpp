#include <dvs_msgs/EventArray.h>

#include <ros/ros.h>
#include <rosbag/message_instance.h>

#include <ctime>

#include "comb/bag_player.h"
#include "comb/comb.h"
#include "comb/utils.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char* argv[]){

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "comb_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  comb::Comb_filter comb_filter(nh, nh_private);

  std::string bag_path;
  nh_private.getParam("bag_path", bag_path);

  bool realtime = bag_path.empty(); // used to determine whether to use realtime or offline mode

  if (realtime)
  {
    // subscriber queue size
    VLOG(1) << "Real-time mode:";
    constexpr int EVENT_SUB_QUEUE_SIZE = 1000;
    ros::Subscriber event_sub = nh.subscribe(
        "events", EVENT_SUB_QUEUE_SIZE, &comb::Comb_filter::eventsCallback,
        &comb_filter);

    ros::spin();
  }
  else if (!realtime)
  {
    std::string working_dir;
    nh_private.getParam("working_dir", working_dir);

    bag_path = comb::utils::fullpath(working_dir, bag_path);

    VLOG(1) << "Path to rosbag: " << bag_path;

    std::string event_topic_name = comb::utils::find_event_topic(bag_path);

    VLOG(1) << "Reading events from topic: " << event_topic_name;

    // attach relevant callbacks to topics
    rpg_common_ros::BagPlayer player(bag_path);
    VLOG(1) << "Bag player";
    player.attachCallbackToTopic(event_topic_name,
        [&](const rosbag::MessageInstance& msg){
          dvs_msgs::EventArray::ConstPtr events = msg.instantiate<dvs_msgs::EventArray>();
          CHECK(events);
          comb_filter.eventsCallback(events);
        }
    );

//    std::clock_t start;
//    double duration;
//
//    start = std::clock();


    player.play();

    VLOG(1) << "Finish.";

//
//    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
//
//    std::cout<< "printf: " << duration <<'\n';

  }

  ros::shutdown();
  return 0;
}
