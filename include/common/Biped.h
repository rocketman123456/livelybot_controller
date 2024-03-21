#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include "common/cppTypes.h"

#include <vector>

class Biped
{
public:
    void setBiped()
    {

        mass = 13.856;
        leg_offset_x2 = 0.0; // 0.0;
        leg_offset_y2 = 0.075;
        leg_offset_z2 = -0.037; //-0.136;
    }
    double leg_offset_x2;
    double leg_offset_y2;
    double leg_offset_z2;
    double mass;

    Vec3<double> getHip2Location(int leg)
    {
        assert(leg >= 0 && leg < 2);
        Vec3<double> pHip2 = Vec3<double>::Zero();
        if (leg == 0)
        {
            pHip2(0) = leg_offset_x2;
            pHip2(1) = leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        if (leg == 1)
        {
            pHip2(0) = leg_offset_x2;
            pHip2(1) = -leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        return pHip2;
    };
};

#endif
