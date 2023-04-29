//
// Created by soulde on 2023/3/4.
//

#include "SerialFrameDealer.h"
#include "CRC/CRC.h"
#include "Serial.h"
//#include "pub_msgs/feedback.h"
//#include "pub_msgs/locate.h"

SerialFrameDealer::SerialFrameDealer(ros::NodeHandle &nh) : nodeHandle(nh) {
    nh.getParam("/serial/subTopic", subName);
    nh.getParam("/serial/goalTopic", goalTopicName);
    nh.getParam("/serial/serialName", serialName);
    serial.reset(new Serial(serialName.c_str()));

    goalPublisher = nh.advertise<geometry_msgs::PoseStamped>(goalTopicName, 1);
    uwbPublisher = nh.advertise<geometry_msgs::PoseStamped>("/UWB", 1);
    enemyClient = nh.serviceClient<std_srvs::SetBool>("/enemy");
    buffClient = nh.serviceClient<std_srvs::SetBool>("/buff");

    twistSubscriber = nh.subscribe<geometry_msgs::Twist>(subName, 1,
                                                         boost::bind(&SerialFrameDealer::twistCallback, this, _1));

    recvThread.reset(new std::thread(&SerialFrameDealer::serialRecv, this));
}

void SerialFrameDealer::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame.header), sizeof(Header));

    moveControlFrame.x = static_cast<float>(msg->linear.x);
    moveControlFrame.y = static_cast<float>(msg->linear.y);
    moveControlFrame.yaw = static_cast<float>(msg->angular.x);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame.x),
                          sizeof(MoveControlFrame) - sizeof(Header));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));
    std::cout << ret << std::endl;
    for (int i = 0; i < sizeof(MoveControlFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&moveControlFrame) + i));
    }


}

[[noreturn]] void SerialFrameDealer::serialRecv() {
    Header header;
    while (true) {
        // wait until head equals HEAD
        while (true) {
            serial->Recv(reinterpret_cast<unsigned char *>(&header), 1);
            if (header.head == HEAD) {
                break;
            }
        }


        serial->Recv(reinterpret_cast<unsigned char *>(&header + 1), sizeof(Header) - 1);
        if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&header), sizeof(Header))) {
            switch (static_cast<RecvPackageID>(header.id)) {
                case RecvPackageID::GIMBAL:
                    serial->Recv(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame), sizeof(GimbalFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame),
                                              sizeof(GimbalFeedbackFrame))) {
                        geometry_msgs::PoseStamped poseStamped;

                        gimbalFeedbackFrame.yaw;
                    }
                    break;
                case RecvPackageID::ODOM:
                    serial->Recv(reinterpret_cast<unsigned char *>(&odomFeedbackFrame), sizeof(OdomFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&odomFeedbackFrame),
                                              sizeof(OdomFeedbackFrame))) {
                        // NOT IMPLEMENT
                    }
                    break;

                case RecvPackageID::GOAL: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&goalFeedbackFrame), sizeof(GoalFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&goalFeedbackFrame),
                                              sizeof(GoalFeedbackFrame))) {

                        geometry_msgs::PoseStamped goal;
                        goal.pose.position.x = goalFeedbackFrame.x;
                        goal.pose.position.y = goalFeedbackFrame.y;
                        goalPublisher.publish(goal);
                    }
                }
                    break;
                case RecvPackageID::ENEMY: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame), sizeof(enemyFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame),
                                              sizeof(enemyFeedbackFrame))) {
                        std_srvs::SetBool enemy;
                        enemy.request.data = enemyFeedbackFrame.isRed;
                        enemyClient.call(enemy);
                    }

                }
                    break;
                case RecvPackageID::UWB: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame), sizeof(UWBFeedbackFrame));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame),
                                              sizeof(UWBFeedbackFrame))) {

                        geometry_msgs::PoseStamped uwb;
                        uwb.pose.position.x = uwbFeedbackFrame.x;
                        uwb.pose.position.y = uwbFeedbackFrame.y;
                        uwbPublisher.publish(uwb);
                    }
                }
                    break;
                case RecvPackageID::BUFF: {
                    std_srvs::SetBool buff;
                    buff.request.data = true;
                    buffClient.call(buff);
                }
                    break;
            }
        }
    }
}



