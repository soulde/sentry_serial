//
// Created by soulde on 2023/3/4.
//

#ifndef SERIAL_SERIALFRAMEDEALER_H
#define SERIAL_SERIALFRAMEDEALER_H

#include <thread>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "Serial.h"


class SerialFrameDealer {

private:
    static constexpr unsigned char HEAD = 0xFE;

    enum class SendPackageID: unsigned char{
        GIMBAL = 0x01,
        MOVE = 0x02

    };

    enum class RecvPackageID: unsigned char{
        GIMBAL = 0x01,
        LOCATE=0x81
    };

#pragma pack(1)
    struct Header {
        const unsigned char head = HEAD;
        unsigned char id = 0x00;
        unsigned char crc8 = 0x00;
    };

    struct SendMoveDataFrame {
        Header header;
        float x = 0.;
        float y = 0.;
        unsigned char rotate = 0;
        unsigned char crc8 = 0x0;
        SendMoveDataFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::MOVE);
        }

    } sendMoveDataFrame;

    struct SendGimbalDataFrame {
        Header header;
        float pitch = 0.;
        float yaw = 0.;
        unsigned char fire = 0;
        unsigned char crc8 = 0x0;
        SendGimbalDataFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::GIMBAL);
        }
    } gimbalDataFrame;

    struct RecvGimbalDataFrame {
        float pitch = 0.;
        float yaw = 0.;
        float fire = 0.;
        unsigned char crc8 = 0x0;
    } recvGimbalDataFrame;

    struct RecvLocateDataFrame {
        float x = 0.;
        float y = 0.;
        unsigned char crc8 = 0x0;
    } recvLocateDataFrame;

#pragma pack()
    Serial* serial;

    std::string subName, pubName, serialName;
    ros::NodeHandle nodeHandle;

    ros::Publisher pub;
    ros::Subscriber subscriber;

    std::thread recvThread;

    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void serialRecv();

public:
    SerialFrameDealer() = delete;
    explicit SerialFrameDealer(ros::NodeHandle &nh);
    ~SerialFrameDealer(){
        delete serial;
    }
    SerialFrameDealer(SerialFrameDealer &serialFrameDealer) = delete;

};


#endif //SERIAL_SERIALFRAMEDEALER_H
