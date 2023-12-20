#ifndef _JACOBIA_H_
#define _JACOBIA_H_
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
// #include <robot_state.h>

class jacobia
{
private:
    // 加载机器人模型
    robot_state::RobotStatePtr *kinematic_state;
    robot_model::RobotModelPtr kinematic_model;
    // 创建机器人状态
    robot_state::JointModelGroup *left_joint_model_group;
    robot_state::JointModelGroup *right_joint_model_group;
    // const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    // 指定要计算雅可比矩阵的链接和参考点
    Eigen::Vector3d reference_point_position;
    Eigen::MatrixXd left_jacobian, right_jacobian;
    std::string right_link_name, left_link_name;

public:
    jacobia()
    {
        robot_model_loader::RobotModelLoader a("robot_description");
        kinematic_model = a.getModel();
        kinematic_state = new robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
        (*kinematic_state)->setToDefaultValues();

        right_joint_model_group = kinematic_model->getJointModelGroup("right_leg_force");
        left_joint_model_group = kinematic_model->getJointModelGroup("left_leg_force");
        right_link_name = right_joint_model_group->getLinkModelNames().back(); // 假设我们想要最后一个链接的雅可比矩阵
        left_link_name = left_joint_model_group->getLinkModelNames().back();   // 假设我们想要最后一个链接的雅可比矩阵

        reference_point_position = Eigen::Vector3d(0.07, 0.0, 0.0);
    }
    ~jacobia()
    {
    }
    void getJacobian(std::vector<std::vector<double>> &joint_values)
    {
        (*kinematic_state)->setJointGroupPositions(left_joint_model_group, joint_values[0]);
        (*kinematic_state)->setJointGroupPositions(right_joint_model_group, joint_values[1]);
        (*kinematic_state)->update();
        // std::cout << "get position:\n";
        // int size = kinematic_model->getJointModelNames().size();
        // std::cout << "size:" << size << "\n"
        //           << std::endl;
        // int cont=0;
        // for (std::string s : kinematic_model->getJointModelNames())
        // {
        //     cont++;
        //     std::cout << cont << " " << s << ": " << *((*kinematic_state)->getJointPositions(s)) << "\n";
        // }
        // std::cout << "ok 1\n";

        (*kinematic_state)->getJacobian(left_joint_model_group, (*kinematic_state)->getLinkModel(left_link_name), reference_point_position, left_jacobian);
        (*kinematic_state)->getJacobian(right_joint_model_group, (*kinematic_state)->getLinkModel(right_link_name), reference_point_position, right_jacobian);
        // std::cout << "ok 2\n";
    }
    Eigen::MatrixXd getLeft_torque(Eigen::VectorXd col)
    {
        return left_jacobian.transpose()*col;
    }
    Eigen::MatrixXd getRight_torque(Eigen::VectorXd col)
    {
        return right_jacobian.transpose()*col;
    }
};

#endif