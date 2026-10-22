#ifndef USV_TEST_EMPIRICAL_PLANT_H
#define USV_TEST_EMPIRICAL_PLANT_H

#include <cmath>
#include <deque>

// Test-only input/output plant. Its parameters describe GPS/compass response,
// including observation latency; they are not separately measured hull forces.
struct EmpiricalPlant {
    float yawTau=2,yawGain=26,yawBias=0,velocityTau=8,surgeGain=.63f;
    float driftEast=0,driftNorth=0,velocityDelay=1.5f;
    double east=0,north=0,heading=90,rate=0,ve=0,vn=0;
    struct Force {float age,east,north;};
    std::deque<Force> forces;
    float clock=0;
    static double decay(double state,double target,double tau,double dt) {
        return target+(state-target)*std::exp(-dt/tau);
    }
    static double integral(double state,double target,double tau,double dt) {
        return target*dt+(state-target)*tau*(-std::expm1(-dt/tau));
    }
    void tick(int left,int right,float dt=.05f) {
        const double diff=(left-right)/356.0,common=(left+right)/356.0;
        const double yawTarget=yawGain*diff+yawBias;
        const double radians=heading*3.141592653589793/180;
        forces.push_back({clock,(float)(common*std::sin(radians)),(float)(common*std::cos(radians))});
        float fe=0,fn=0;
        while(!forces.empty()&&forces.front().age<=clock-velocityDelay+1e-4f) {
            fe=forces.front().east;fn=forces.front().north;forces.pop_front();
            lastEast=fe;lastNorth=fn;
        }
        fe=lastEast;fn=lastNorth;
        const double targetE=driftEast+surgeGain*fe,targetN=driftNorth+surgeGain*fn;
        east+=integral(ve,targetE,velocityTau,dt);north+=integral(vn,targetN,velocityTau,dt);
        ve=decay(ve,targetE,velocityTau,dt);vn=decay(vn,targetN,velocityTau,dt);
        heading+=integral(rate,yawTarget,yawTau,dt);rate=decay(rate,yawTarget,yawTau,dt);
        heading=std::fmod(heading+3600,360.0);clock+=dt;
    }
private:
    float lastEast=0,lastNorth=0;
};
#endif
