#ifndef _RobotState
#define _RobotState

#include "ConvexMPC/common_types.h"

#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,2> r_feet;
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
        //fpt m = 19; // Aliengo
        fpt m = 13; // A1
    //private:
};
#endif
