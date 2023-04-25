//
// Created by soulde on 2023/3/4.
//

#include "SerialFrameDealer.h"
#include "CRC/CRC.h"
#include "Serial.h"
//#include "pub_msgs/feedback.h"
//#include "pub_msgs/locate.h"

SerialFrameDealer::SerialFrameDealer(ros::NodeHandle &nh) : nodeHandle(nh),
                                                            recvThread(&SerialFrameDealer::serialRecv, this) {
    nh.getParam("/serial/subTopic", subName);
    nh.getParam("/serial/pubTopic", pubName);
    nh.getParam("/serial/serialName", serialName);
    serial = new Serial(serialName.c_str());



//    publisher = nh.advertise<std_msgs::Empty>(pub_name, 1);
//    pub = nh.advertise<pub_msgs::feedback>(pubName, 1);
    subscriber = nh.subscribe<geometry_msgs::Twist>(subName, 1,
                                                    boost::bind(&SerialFrameDealer::twistCallback, this, _1));

}

void SerialFrameDealer::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&sendMoveDataFrame.header), sizeof(Header));

    sendMoveDataFrame.x = static_cast<float>(msg->linear.x);
    sendMoveDataFrame.y = static_cast<float>(msg->linear.y);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&sendMoveDataFrame.x),
                          sizeof(SendMoveDataFrame) - sizeof(Header));

    int32_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&sendMoveDataFrame), sizeof(SendMoveDataFrame));
    std::cout << ret << std::endl;
    for (int i = 0; i < sizeof(SendMoveDataFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&sendMoveDataFrame) + i));
    }
    std::cout << std::endl;
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&gimbalDataFrame.header), sizeof(Header));

    gimbalDataFrame.yaw = static_cast<float>(msg->angular.x);
    gimbalDataFrame.pitch = static_cast<float>(msg->linear.y);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&gimbalDataFrame.yaw),
                          sizeof(SendGimbalDataFrame) - sizeof(Header));

    serial->Send(reinterpret_cast<const unsigned char *>(&gimbalDataFrame), sizeof(SendGimbalDataFrame));
}

void SerialFrameDealer::serialRecv() {
//    Header header;
//    while (true) {
//        serial.Read(reinterpret_cast<unsigned char *>(&header), 1);
//        if (header.head == HEAD) {
//            break;
//        }
//    }
//
//    serial.Read(reinterpret_cast<unsigned char *>(&header + 1), sizeof(Header) - 1);
//    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&header), sizeof(Header))) {
//        switch (static_cast<RecvPackageID>(header.id)) {
//            case RecvPackageID::GIMBAL:
//                serial.Read(reinterpret_cast<unsigned char *>(&recvGimbalDataFrame), sizeof(RecvGimbalDataFrame));
//                if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&recvGimbalDataFrame),
//                                          sizeof(RecvGimbalDataFrame))) {
//
//                    pub_msgs::feedback msg;
//                    msg.Pitch = recvGimbalDataFrame.pitch;
//                    msg.Yaw = recvGimbalDataFrame.yaw;
//                    msg.Fire = recvGimbalDataFrame.fire;
//                    pub.publish(msg);
//
//                }
//                break;
//            case RecvPackageID::LOCATE:
//                serial.Read(reinterpret_cast<unsigned char *>(&recvLocateDataFrame), sizeof(RecvLocateDataFrame));
//                if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&recvLocateDataFrame),
//                                          sizeof(RecvLocateDataFrame))) {
//
//                    pub_msgs::locate msg;
//                    msg.X = recvLocateDataFrame.x;
//                    msg.Y = recvLocateDataFrame.y;
//                    pub.publish(msg);
//                }
//                break;
//
//        }
//    }
}
