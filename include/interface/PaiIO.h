#ifndef _PAIIO_H_
#define _PAIIO_H_

#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include "livelybot_msg/MotorCmd.h"
#include "livelybot_msg/MotorState.h"
#include "livelybot_msg/HighState.h"
#include "livelybot_msg/LowCmd.h"

#include "IOInterface.h"
#include "KeyBoard.h"
#include "../use.h"

#include "hardware/robot.h"
// #define USE 1 // 0是Gazebo 1是真实机器人

#include <csignal>
class PaiIO : public IOInterface
{
private:
    ros::NodeHandle _nm;

    std::string _robot_name;
    livelybot_msg::HighState _highState;
    livelybot_msg::LowCmd _lowCmd;
    double f;
public:
#if USE // 使用真实机器人
    // send_recv _send_recv;
    robot rb;
#else
    ros::Subscriber _servo_sub[10], _state_sub;
    ros::Publisher _servo_pub[10];
#endif
    PaiIO(std::string robot_name, const std::string spi_name,double dt);
    ~PaiIO();
    void sendCmd(const LowlevelCmd *cmd);
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    void recvState(LowlevelState *state);
#if USE // 使用真实机器人
#else   // 使用Gazebo
    void initSend();
    void initRecv();
    void StateCallback(const gazebo_msgs::ModelStates &msg);
    void LhipCallback(const livelybot_msg::MotorState &msg);
    void Lhip2Callback(const livelybot_msg::MotorState &msg);
    void LthighCallback(const livelybot_msg::MotorState &msg);
    void LcalfCallback(const livelybot_msg::MotorState &msg);
    void LtoeCallback(const livelybot_msg::MotorState &msg);
    void RhipCallback(const livelybot_msg::MotorState &msg);
    void Rhip2Callback(const livelybot_msg::MotorState &msg);
    void RthighCallback(const livelybot_msg::MotorState &msg);
    void RcalfCallback(const livelybot_msg::MotorState &msg);
    void RtoeCallback(const livelybot_msg::MotorState &msg);
#endif
};

#endif