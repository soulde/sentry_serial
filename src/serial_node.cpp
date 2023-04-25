#include "SerialFrameDealer.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    SerialFrameDealer dealer(nh);
    while(ros::ok()){
        ros::spinOnce();
    }
}