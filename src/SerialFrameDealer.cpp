//
// Created by soulde on 2023/3/4.
//

#include "SerialFrameDealer.h"
#include "CRC/CRC.h"
#include "Serial.h"
#include <eigen_conversions/eigen_msg.h>

SerialFrameDealer::SerialFrameDealer(ros::NodeHandle &nh) : nodeHandle(nh) {
    nh.getParam("/sentry_serial/serialName", serialName);

    nh.getParam("/sentry_serial/subTopic", subTopic);
    nh.getParam("/sentry_serial/goalTopic", goalTopic);
    nh.getParam("/sentry_serial/GNSSTopic", GNSSTopic);

    nh.getParam("/sentry_serial/fixFrame", fixFrame);
    nh.getParam("/sentry_serial/robotFrame", robotFrame);
    nh.getParam("/sentry_serial/gimbalFrame", gimbalFrame);

    nh.getParam("/sentry_serial/base2gimbal_x", base2gimbalTrans.x());
    nh.getParam("/sentry_serial/base2gimbal_y", base2gimbalTrans.y());
    nh.getParam("/sentry_serial/base2gimbal_z", base2gimbalTrans.z());


    serial = new Serial(serialName.c_str());


    odom.header.seq = 0;
    odom.header.frame_id = fixFrame;
    odom.child_frame_id = odomFrame;


    base2gimbal.header.seq = 0;
    base2gimbal.header.frame_id = robotFrame;
    base2gimbal.child_frame_id = gimbalFrame;

    world2base.header.seq = 0;
    world2base.header.frame_id = fixFrame;
    world2base.child_frame_id = robotFrame;

    goal.header.seq = 0;
    goal.header.frame_id = fixFrame;

    uwb.header = goal.header;

    odomPublisher = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    goalPublisher = nh.advertise<geometry_msgs::PoseStamped>(goalTopic, 1);
    uwbPublisher = nh.advertise<geometry_msgs::PoseStamped>("/UWB", 1);
    enemyClient = nh.serviceClient<std_srvs::SetBool>("/enemy");
    buffClient = nh.serviceClient<std_srvs::SetBool>("/buff");

    twistSubscriber = nh.subscribe<geometry_msgs::Twist>(subTopic, 1,
                                                         [this](const geometry_msgs::Twist::ConstPtr &msg) {
                                                             this->twistCallback(msg);
                                                         });

    GNSSSubscriber = nh.subscribe<sensor_msgs::NavSatFix>(GNSSTopic, 1,
                                                          [this](const sensor_msgs::NavSatFix::ConstPtr &msg) {
                                                              this->GNSSCallback(msg);
                                                          });
    recvThread = new std::thread(&SerialFrameDealer::serialRecv, this);
}

void SerialFrameDealer::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame.header), sizeof(Header));

    moveControlFrame.x = static_cast<float>(msg->linear.x);
    moveControlFrame.y = static_cast<float>(msg->linear.y);
    moveControlFrame.yaw = static_cast<float>(msg->angular.z);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));

    for (int i = 0; i < sizeof(MoveControlFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&moveControlFrame) + i));
    }
    std::cout << std::endl;
}

void SerialFrameDealer::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&GNSSFrame.header), sizeof(Header));
    std::cout << "GNSS Data Send" << std::endl;
    GNSSFrame.lon = static_cast<float>(msg->longitude);
    GNSSFrame.lat = static_cast<float>(msg->latitude);
    GNSSFrame.alt = static_cast<float>(msg->altitude);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&GNSSFrame), sizeof(GNSSFrame));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&GNSSFrame), sizeof(GNSSFrame));

    for (int i = 0; i < sizeof(GNSSFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&GNSSFrame) + i));
    }
    std::cout << std::endl;
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
        serial->Recv(reinterpret_cast<unsigned char *>(&header) + 1, sizeof(Header) - 1);
        if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&header), sizeof(Header))) {

            switch (static_cast<RecvPackageID>(header.id)) {
                case RecvPackageID::GIMBAL: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame) + sizeof(Header),
                                 sizeof(GimbalFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame),
                                              sizeof(GimbalFeedbackFrame))) {
                        base2gimbal = tf2::eigenToTransform(
                                base2gimbalTrans *
                                Eigen::AngleAxisd(gimbalFeedbackFrame.pitch / 180 * M_PI, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(gimbalFeedbackFrame.yaw / 180 * M_PI, Eigen::Vector3d::UnitZ())
                        );

                        base2gimbal.header.seq++;
                        base2gimbal.header.stamp = ros::Time::now();

                        broadcaster.sendTransform(base2gimbal);
                    }
                }
                    break;
                case RecvPackageID::ODOM: {
                    static uint32_t count;
                    serial->Recv(reinterpret_cast<unsigned char *>(&odomFeedbackFrame) + sizeof(Header),
                                 sizeof(OdomFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&odomFeedbackFrame),
                                              sizeof(OdomFeedbackFrame))) {
                        Eigen::Affine3d affinePos = Eigen::Translation3d(odomFeedbackFrame.x, odomFeedbackFrame.y, 0) *
                                                    Eigen::AngleAxisd(odomFeedbackFrame.yaw / 180 * M_PI,
                                                                      Eigen::Vector3d::UnitZ());
                        Eigen::Matrix<double, 6, 1> twist;
                        twist << odomFeedbackFrame.vx, odomFeedbackFrame.vy, 0, 0, 0, odomFeedbackFrame.wy;

                        tf::poseEigenToMsg(affinePos, odom.pose.pose);
                        tf::twistEigenToMsg(twist, odom.twist.twist);

                        odom.header.seq++;
                        odom.header.stamp = ros::Time::now();

                        odomPublisher.publish(odom);

                        world2base = tf2::eigenToTransform(affinePos);
                        world2base.header.seq++;
                        world2base.header.stamp = ros::Time::now();

                        broadcaster.sendTransform(world2base);
                    }

                }
                    break;
                case RecvPackageID::GOAL: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&goalFeedbackFrame) + sizeof(Header),
                                 sizeof(GoalFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&goalFeedbackFrame),
                                              sizeof(GoalFeedbackFrame))) {
                        goal.pose.position.x = goalFeedbackFrame.x;
                        goal.pose.position.y = goalFeedbackFrame.y;
                        goal.header.seq++;

                        goalPublisher.publish(goal);
                    }
                }
                    break;
                case RecvPackageID::ENEMY: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame) + sizeof(Header),
                                 sizeof(enemyFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame),
                                              sizeof(enemyFeedbackFrame))) {
                        std_srvs::SetBool enemy;
                        enemy.request.data = enemyFeedbackFrame.isRed;

                        enemyClient.call(enemy);
                    }

                }
                    break;
                case RecvPackageID::UWB: {
                    serial->Recv(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame) + sizeof(Header),
                                 sizeof(UWBFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame),
                                              sizeof(UWBFeedbackFrame))) {
                        uwb.pose.position.x = uwbFeedbackFrame.x;
                        uwb.pose.position.y = uwbFeedbackFrame.y;
                        uwb.header.stamp = ros::Time::now();
                        uwb.header.seq++;

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





