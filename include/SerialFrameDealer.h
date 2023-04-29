//
// Created by soulde on 2023/3/4.
//

#ifndef SERIAL_SERIALFRAMEDEALER_H
#define SERIAL_SERIALFRAMEDEALER_H
#define float32_t float
#define float64_t double
#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int8.h"
#include "std_srvs/SetBool.h"


#include "Serial.h"


class SerialFrameDealer {

private:
    static constexpr unsigned char HEAD = 0xFE;

    enum class SendPackageID: unsigned char{
        GIMBAL = 0x01,
        MOVE = 0x02,
        HEARTBEAT = 0x41

    };

    enum class RecvPackageID: unsigned char{
        GIMBAL = 0x81,
        ODOM=0x82,
        GOAL=0x83,
        ENEMY=0x84,
        UWB=0x85,
        BUFF=0xC1
    };

#pragma pack(1)
    struct Header {
        const uint8_t head = HEAD;
        uint8_t id = 0x00;
        uint8_t crc8 = 0x00;
    };

    struct MoveControlFrame {
        Header header;
        float32_t x = 0.;
        float32_t y = 0.;
        float32_t yaw = 0.;
        uint8_t crc8 = 0x0;
        MoveControlFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::MOVE);
        }

    } moveControlFrame;

    struct GimbalControlFrame {
        Header header;
        float32_t pitch = 0.;
        float32_t yaw = 0.;
        uint8_t fire = 0;
        uint8_t crc8 = 0x0;
        GimbalControlFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::GIMBAL);
        }
    } gimbalControlFrame;

    struct HeartBeatFrame{
        Header header;
        HeartBeatFrame(){
            header.id = static_cast<unsigned char>(SendPackageID::HEARTBEAT);
        }
    }heartBeatFrame;

    struct GimbalFeedbackFrame {
        float32_t pitch = 0.;
        float32_t yaw = 0.;
        float32_t fire = 0.;
        uint8_t crc8 = 0x0;
    } gimbalFeedbackFrame;

    struct OdomFeedbackFrame {
        float32_t x = 0.;
        float32_t y = 0.;
        uint8_t crc8 = 0x0;
    } odomFeedbackFrame;

    struct GoalFeedbackFrame{
        float32_t x;
        float32_t y;
    }goalFeedbackFrame;

    struct EnemyFeedbackFrame{
        uint8_t isRed;
    }enemyFeedbackFrame;

    struct UWBFeedbackFrame{
        float32_t x;
        float32_t y;
    }uwbFeedbackFrame;
#pragma pack()
    std::unique_ptr<Serial> serial;

    std::string subName, goalTopicName, serialName;
    ros::NodeHandle nodeHandle;

    ros::Publisher goalPublisher, uwbPublisher;
    ros::Subscriber twistSubscriber;

    ros::ServiceClient enemyClient, buffClient;

    std::unique_ptr<std::thread> recvThread;

    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);

    [[noreturn]] void serialRecv();

public:
    SerialFrameDealer() = delete;
    explicit SerialFrameDealer(ros::NodeHandle &nh);
    SerialFrameDealer(SerialFrameDealer &serialFrameDealer) = delete;

};


#endif //SERIAL_SERIALFRAMEDEALER_H
