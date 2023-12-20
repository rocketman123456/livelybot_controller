#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <stdlib.h>
#include "../include/interface/PaiIO.h"
#include "../include/common/ControlFSMData.h"
#include "../include/common/OrientationEstimator.h"
#include "../include/common/PositionVelocityEstimator.h"
#include "../include/FSM/FSM.h"
#include <ros/ros.h>
#include "jacobia.h"
using namespace std;
// 一个简单的demo，实现机器人站起来/蹲下的操作
bool running = true;
#define PI 3.1415926
void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
    system("stty sane"); // Terminal back to normal
    exit(0);
}
void set_motor(ros::NodeHandle nh)
{
    std::vector<std::string> _motor_names, side;
    _motor_names.push_back("_hip");
    _motor_names.push_back("_hip2");
    _motor_names.push_back("_thigh");
    _motor_names.push_back("_calf");
    _motor_names.push_back("_toe");
    side.push_back("L");
    side.push_back("R");
    int ID[5] = {5, 4, 3, 2, 1};
    int CAN[2] = {0x10, 0x20};
    int num[5] = {0, 1, 2, 3, 4};
    int count_side = 0;
    int count_motor = 0;
    for (std::string _side : side)
    {
        for (std::string _motor_name : _motor_names)
        {
            std::cout << _side + _motor_name << std::endl;
            nh.setParam(_side + _motor_name + "_ID", CAN[count_side] | ID[count_motor]);
            nh.setParam(_side + _motor_name + "_num", num[count_motor] + count_side * 5);
            count_motor++;
        }
        count_motor = 0;
        count_side++;
    }
}
int main(int argc, char **argv)
{
    IOInterface *ioInter;
    ros::init(argc, argv, "pai_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    set_motor(nh);
    ros::Rate rate(1000);
    double dt = 0.001;
    std::string robot_name = "livelybot";
    // std::cout << "robot name " << robot_name << std::endl;
    Biped biped;
    StateEstimate stateEstimate;
    biped.setBiped();
    // PaiIO pai(robot_name, "/dev/spidev4.1");
    ioInter = new PaiIO(robot_name, "/dev/spidev4.1", dt);

    LowlevelCmd *cmd = new LowlevelCmd();
    LowlevelState *state = new LowlevelState();
    std::cout << "init ok" << std::endl;
    signal(SIGINT, ShutDown);
    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////jocbia////////////////////////////////////////////////
    // jacobia left_j("left_leg_force");
    jacobia j;
    Eigen::MatrixXd left_jacobia;
    Eigen::MatrixXd right_jacobia;
    std::vector<double> joint_value_l, joint_value_r;
    std::vector<std::vector<double>> joint_value;
    for (size_t i = 0; i < 5; i++)
    {
        joint_value_l.push_back(0.0);
    }
    for (size_t i = 0; i < 5; i++)
    {
        joint_value_r.push_back(0.0);
    }
    joint_value.push_back(joint_value_l);
    joint_value.push_back(joint_value_r);
    double Fz, hd, h, old_h, Kz, d_hd, d_h, d_Kz;
    old_h = hd = h = 0.3825;
    Kz = 900;
    d_Kz = 40;
    double derta = 0.00002; // 2cm/s
    double derta_t = 0.001;
    d_hd = -derta / derta_t;
    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////simple control////////////////////////////////////////
    for (size_t i = 0; i < 10; i++)
    {
        cmd->motorCmd[i].q = 0.0;
        cmd->motorCmd[i].dq = 0.0;
        cmd->motorCmd[i].tau = 0.0;
        cmd->motorCmd[i].Kp = 0.0; // 50.0;
        cmd->motorCmd[i].Kd = 0.0; // 0.010;
    }
    int cont = 0;
    int cont_2 = 0;
    while (ros::ok())
    {
        /////////////////////////////fresh target hight///////////////////
        /////////////////////////////calculate F/T ///////////////////////
        //  因为机old_h心在脚底板中心正上方,Fx=0,Fy=0,Tx=0,Ty=0,Tz=0
        //  J(^T)*[Fx Fy Fz Tx Ty Tz](^T) = torque
        printf("=====================================\n");
        // hd -= derta;
        cont++;
        if (cont == 1000)
            derta *= -1;
        cont = 0;
        h = ioInter->get_now_z();
        d_h = (h - old_h) / derta_t;
        Fz = Kz * (h - hd) -6;//- d_Kz * (d_h - d_hd) ;
        old_h = h;
        std::cout<<Kz * (h - hd)<<" "<<- d_Kz * (d_h - d_hd)<<std::endl;
        std::cout << "now pos:" << h << " now vel:" << d_h << "\n"
                  << "target pos:" << hd << " target vel:" << d_hd << "\n";
        std::cout << Fz << "\n";
        Eigen::VectorXd colVector(6);
        colVector << 0.0, 0.0, Fz, 0.0, 0.0, 0.0;

        for (size_t i = 0; i < 5; i++)
        {
            joint_value[0][i] = state->motorState[i].q;
            joint_value[1][i] = state->motorState[i + 5].q;
        }
        j.getJacobian(joint_value);

        // std::cout << "the state pos\n";
        // for (size_t i = 0; i < 10; i++)
        // {
        //     std::cout << state->motorState[i].q << " ";
        // }
        // std::cout << "\n";

        Eigen::VectorXd right_motor_torque = j.getRight_torque(colVector);
        Eigen::VectorXd left_motor_torque = j.getLeft_torque(colVector);
        // std::cout << "right Jacobian: \n"
        //           << right_jacobia << std::endl;
        // std::cout << "left Jacobian: \n"
        //           << left_jacobia << std::endl;
        std::cout << "right torque:\n"
                  << right_motor_torque << std::endl;
        std::cout << "left torque:\n"
                  << left_motor_torque << std::endl;
        if (cont_2 < 5)
        {
            cont_2++;
        }
        else
        {
            for (size_t i = 0; i < 5; i++)
            {
                cmd->motorCmd[i].tau = left_motor_torque[i];
                cmd->motorCmd[i + 5].tau = right_motor_torque[i];
            }
            // cmd->motorCmd[2].tau*=1.2;
            // cmd->motorCmd[7].tau*=1.2;
        }
        ioInter->sendRecv(cmd, state);
        rate.sleep();
    }

    ////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////gazebo////////////////////////////////////////////////

    LegController *legController = new LegController(biped);
    StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(state,
                                                                          legController->data,
                                                                          &stateEstimate);
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    DesiredStateCommand *desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData *_controlData = new ControlFSMData;
    _controlData->_biped = &biped;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;
    _controlData->_interface = ioInter;
    _controlData->_lowCmd = cmd;
    _controlData->_lowState = state;
    FSM *_FSMController = new FSM(_controlData);
    while (ros::ok())
    {
        _FSMController->run();
        rate.sleep();
    }
    delete _controlData;
    return 0;
}
