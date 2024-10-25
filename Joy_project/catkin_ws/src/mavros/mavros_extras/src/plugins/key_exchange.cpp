#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <std_msgs/UInt8MultiArray.h>  // 수정된 메시지 타입
#include <array>
#include <ros/ros.h>

#include <random>
#include <monocypher.h>

#include <cstdio>

#define KEY_SIZE 32

namespace mavros {
namespace extra_plugins {

class KeyExchangePlugin : public plugin::PluginBase {
public:
    KeyExchangePlugin() : PluginBase(),
        nh("~key_exchange")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        pub = nh.advertise<std_msgs::UInt8MultiArray>("key_exchange_pub", 10);  // 퍼블리셔 생성
        timer = nh.createTimer(ros::Duration(10.0), &KeyExchangePlugin::send_key_exchange, this);
    };

    Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Timer timer;
    int rate;
    uint8_t public_key[KEY_SIZE] = {
    	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    
    void key_generation(uint8_t *key) {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_int_distribution<uint8_t> dis(0, 255);

        for(int i=0; i<KEY_SIZE; i++) {
            key[i] = dis(gen);
        }
    }

    void send_key_exchange(const ros::TimerEvent&)
    {
        mavlink::common::msg::KEY_EXCHANGE kc {};
        memcpy(kc.public_key.data(), public_key, KEY_SIZE);

        // ROS 퍼블리시를 위해 메시지 준비
        std_msgs::UInt8MultiArray ros_msg;
        ros_msg.data.clear();  // 배열 초기화
        ros_msg.data.insert(ros_msg.data.end(), public_key, public_key + KEY_SIZE);  // public_key 배열 복사

        pub.publish(ros_msg);  // 전체 KEY_SIZE바이트 public_key 퍼블리시

        // MAVLink 메시지 전송
        UAS_FCU(m_uas)->send_message_ignore_drop(kc);
    }
};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::KeyExchangePlugin, mavros::plugin::PluginBase)
