#include "../../include/interface/PaiIO.h"
inline void RosShutDown(int sig)
{
    ROS_INFO("ROS interface shutting down!");
    ros::shutdown();
}
PaiIO::PaiIO(std::string robot_name, const std::string spi_name, double dt) : IOInterface(), f(1 / dt)
{
    std::cout << "The control interface for ROS Gazebo simulation with cheat states from gazebo" << std::endl;
    _robot_name = robot_name;
    std::cout << "The robot name: " << robot_name << std::endl;
#if USE
#else
    // start subscriber
    initRecv();
    ROS_INFO("initRecv");
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(3000); // wait for subscribers start
    // initialize publisher
    initSend();
    ROS_INFO("initSend");
#endif
    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}
PaiIO::~PaiIO()
{
    ros::shutdown();
}
void PaiIO::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // std::cout << "=================" << std::endl;
    sendCmd(cmd);
    std::cout<<"paiIO sendRecv 1\n";
#if USE
    int num;
    motor_back_t motor_data;
    for (motor *m : rb.Motors)
    {
        motor_data = *m->get_current_motor_state();
        num = m->get_motor_num() - 1;
        state->motorState[num].q = motor_data.position;
        state->motorState[num].dq = motor_data.velocity;
        state->motorState[num].tauEst = motor_data.torque;
        // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
    }
#else
    recvState(state);
    std::cout<<"paiIO sendRecv 2\n";
#endif

    cmdPanel->updateVelCmd(state);
    std::cout<<"paiIO sendRecv 3\n";
    state->userCmd = cmdPanel->getUserCmd();
    std::cout<<"paiIO sendRecv 4\n";
    state->userValue = cmdPanel->getUserValue();
    std::cout<<"paiIO sendRecv 5\n";
}
void PaiIO::sendCmd(const LowlevelCmd *cmd)
{
#if USE
    int num;
    for (motor *m : rb.Motors)
    {
        num = m->get_motor_num() - 1;
        m->fresh_cmd(cmd->motorCmd[num].q, cmd->motorCmd[num].dq, cmd->motorCmd[num].tau, cmd->motorCmd[num].Kp, cmd->motorCmd[num].Kd);
    }
    rb.motor_send();
#else

    // std::cout<<"paiIO sendCmd 1\n";
    for (int i = 0; i < 10; i++)
    {
        _lowCmd.motorCmd[i].mode = 0X0A; // alwasy set it to 0X0A
        _lowCmd.motorCmd[i].q = cmd->motorCmd[i].q;

        _lowCmd.motorCmd[i].dq = cmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = cmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = cmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = cmd->motorCmd[i].Kp;
    }

    // std::cout<<_lowCmd.motorCmd[6].q<<std::endl;

    // std::cout<<"paiIO sendCmd 2\n";
    for (int m = 0; m < 10; m++)
    {
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    // std::cout<<"paiIO sendCmd 3\n";
#endif
    ros::spinOnce();
}
void PaiIO::recvState(LowlevelState *state)
{

    for (int i = 0; i < 10; i++)
    {
        state->motorState[i].q = _highState.motorState[i].pos;
        state->motorState[i].dq = _highState.motorState[i].vel;
        state->motorState[i].tauEst = _highState.motorState[i].tau;
        std::cout<<i<<" "<<state->motorState[i].q <<"\n";
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _highState.imu.quaternion[i];
        state->imu.gyroscope[i] = _highState.imu.gyroscope[i];
        state->position[i] = _highState.position[i];
        state->vWorld[i] = _highState.velocity[i];
    }
    state->imu.quaternion[3] = _highState.imu.quaternion[3];
}
#if USE // 使用真实机器人
std::string _use = "_real/";
#else   // 使用Gazebo
std::string _use = "_gazebo";
void PaiIO::initSend()
{
    _servo_pub[0] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/L_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/L_hip2_controller/command", 1);
    _servo_pub[2] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/L_thigh_controller/command", 1);
    _servo_pub[3] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/L_calf_controller/command", 1);
    _servo_pub[4] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/L_toe_controller/command", 1);
    _servo_pub[5] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/R_hip_controller/command", 1);
    _servo_pub[6] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/R_hip2_controller/command", 1);
    _servo_pub[7] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/R_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/R_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<livelybot_msg::MotorCmd>("/" + _robot_name + _use + "/R_toe_controller/command", 1);
}
void PaiIO::initRecv()
{
#if USE // 使用真实机器人
    _state_sub = _nm.subscribe("/real/model_states", 1, &PaiIO::StateCallback, this);
#else   // 使用Gazebo
    _state_sub = _nm.subscribe("/gazebo/model_states", 1, &PaiIO::StateCallback, this);
#endif
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + _use + "/L_hip_controller/state", 1, &PaiIO::LhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + _use + "/L_hip2_controller/state", 1, &PaiIO::Lhip2Callback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + _use + "/L_thigh_controller/state", 1, &PaiIO::LthighCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + _use + "/L_calf_controller/state", 1, &PaiIO::LcalfCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + _use + "/L_toe_controller/state", 1, &PaiIO::LtoeCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + _use + "/R_hip_controller/state", 1, &PaiIO::RhipCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + _use + "/R_hip2_controller/state", 1, &PaiIO::Rhip2Callback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + _use + "/R_thigh_controller/state", 1, &PaiIO::RthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + _use + "/R_calf_controller/state", 1, &PaiIO::RcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + _use + "/R_toe_controller/state", 1, &PaiIO::RtoeCallback, this);
}

void PaiIO::StateCallback(const gazebo_msgs::ModelStates &msg)
{
    // std::cout <<"StateCallback "<< std::endl;
    int robot_index = -1;
    std::cout << msg.name.size() << std::endl;
    for (int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == _robot_name + _use)
        {
            robot_index = i;
        }
    }
    if (robot_index > -1)
    {
        _highState.position[0] = msg.pose[robot_index].position.x;
        _highState.position[1] = msg.pose[robot_index].position.y;
        _highState.position[2] = msg.pose[robot_index].position.z;
        _highState.velocity[0] = msg.twist[robot_index].linear.x;
        _highState.velocity[1] = msg.twist[robot_index].linear.y;
        _highState.velocity[2] = msg.twist[robot_index].linear.z;

        _highState.imu.quaternion[0] = msg.pose[robot_index].orientation.w;
        _highState.imu.quaternion[1] = msg.pose[robot_index].orientation.x;
        _highState.imu.quaternion[2] = msg.pose[robot_index].orientation.y;
        _highState.imu.quaternion[3] = msg.pose[robot_index].orientation.z;

        _highState.imu.gyroscope[0] = msg.twist[robot_index].angular.x;
        _highState.imu.gyroscope[1] = msg.twist[robot_index].angular.y;
        _highState.imu.gyroscope[2] = msg.twist[robot_index].angular.z;
    }
    else
    {
        std::cout << "The robot name is incorrect,the initial name is: " << _robot_name + _use<< ",but the name is not on the list" << std::endl;
        for (int i = 0; i < msg.name.size(); i++)
        {
            std::cout << msg.name[i] << " " << _robot_name + _use<< std::endl;
        }
    }
}
void PaiIO::LhipCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[0].pos = msg.pos;
    _highState.motorState[0].vel = msg.vel;
    _highState.motorState[0].tau = msg.tau;
}
void PaiIO::Lhip2Callback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[1].pos = msg.pos;
    _highState.motorState[1].vel = msg.vel;
    _highState.motorState[1].tau = msg.tau;
}
void PaiIO::LthighCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[2].pos = msg.pos;
    _highState.motorState[2].vel = msg.vel;
    _highState.motorState[2].tau = msg.tau;
}
void PaiIO::LcalfCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[3].pos = msg.pos;
    _highState.motorState[3].vel = msg.vel;
    _highState.motorState[3].tau = msg.tau;
}
void PaiIO::LtoeCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[4].pos = msg.pos;
    _highState.motorState[4].vel = msg.vel;
    _highState.motorState[4].tau = msg.tau;
}
void PaiIO::RhipCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[5].pos = msg.pos;
    _highState.motorState[5].vel = msg.vel;
    _highState.motorState[5].tau = msg.tau;
}
void PaiIO::Rhip2Callback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[6].pos = msg.pos;
    _highState.motorState[6].vel = msg.vel;
    _highState.motorState[6].tau = msg.tau;
}
void PaiIO::RthighCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[7].pos = msg.pos;
    _highState.motorState[7].vel = msg.vel;
    _highState.motorState[7].tau = msg.tau;
}
void PaiIO::RcalfCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[8].pos = msg.pos;
    _highState.motorState[8].vel = msg.vel;
    _highState.motorState[8].tau = msg.tau;
}
void PaiIO::RtoeCallback(const livelybot_msg::MotorState &msg)
{
    _highState.motorState[9].pos = msg.pos;
    _highState.motorState[9].vel = msg.vel;
    _highState.motorState[9].tau = msg.tau;
}
#endif
