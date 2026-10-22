#ifndef USV_OPEN_WATER_SCENARIO_RIG_H
#define USV_OPEN_WATER_SCENARIO_RIG_H

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include "control/GridHeadingController.h"
#include "control/HeadingController.h"
#include "control/DriveController.h"
#include "navigation/Navigator.h"
#include "config.h"

namespace OpenWater {
inline float wrap(float v) { v=std::fmod(v,360.0f); return v<0?v+360:v; }
inline Waypoint waypoint(double east,double north) {
    double lat,lon;Navigator::fromLocalEnu(20,99,east,north,lat,lon);
    return Waypoint{lat,lon};
}
struct Input {
    float target=90, heading=90, trueHeading=90;
    float dt=.05f, distance=100, crossTrack=0;
    bool fresh=true, gps=true, hold=false, emergency=false, resetHeading=false;
    BoatState state=STATE_GRID_PATROL;
    SafetyState safety=SafetyState::NORMAL;
    unsigned sequence=0, waypoint=0;
};
struct Row {
    Input in;
    float time=0,error=0,rate=0,steering=0;
    int base=0;
    bool turning=false;
    DriveRequest request{};
    DriveDecision applied{};
};
struct Rig {
    GridHeadingController heading;
    HeadingController legacy{HEADING_KP,HEADING_KI,HEADING_KD};
    DriveController drive{MOTOR_PWM_LIMIT,MOTOR_MIN_PWM,MOTOR_PWM_SLEW_PER_SEC};
    uint32_t us=0;
    Row tick(const Input& in) {
        us+=(uint32_t)std::lround(in.dt*1000000.0f);
        Row row;row.in=in;row.time=us/1000000.0f;
        if(in.resetHeading) { heading.reset();legacy.reset(); }
        row.error=std::isfinite(in.target)&&std::isfinite(in.heading)
            ?HeadingController::computeHeadingError(in.target,in.heading):NAN;
        row.base=std::fabs(row.error)>LOS_TURN_FIRST_DEG?0:
            (in.distance<LOS_NEAR_SLOW_M?MOTOR_APPROACH_PWM:
                (in.state==STATE_GRID_PATROL?GRID_CRUISE_PWM:
                    (in.state==STATE_SPIRAL_MAPPING?MOTOR_APPROACH_PWM:MOTOR_CRUISE_PWM)));
        row.request={DriveSource::NONE,0,0,false};
        const bool active=in.state==STATE_GRID_PATROL || in.state==STATE_SPIRAL_MAPPING || in.state==STATE_RTH;
        const bool available=in.fresh&&in.gps&&std::isfinite(in.heading);
        if(in.state==STATE_GRID_PATROL && available&&!in.hold) {
            row.request=heading.update(in.target,in.heading,row.base,us,in.fresh);
            row.rate=heading.headingRateDegPerSec();row.steering=heading.steeringPwm();
            row.turning=heading.isTurningInPlace();
        } else {
            heading.reset();
            if(active&&available&&!in.hold&&std::isfinite(row.error)) {
                legacy.calculateThrusterOutputs(in.target,in.heading,row.base,in.dt,
                    row.request.leftPwm,row.request.rightPwm);
                row.request.source=in.state==STATE_RTH?DriveSource::RTH:DriveSource::SPIRAL;
                row.request.valid=true;
            }
        }
        DriveInterlocks locks{in.state,in.safety,in.emergency,available,in.hold};
        row.applied=drive.resolve(row.request,locks,in.dt);
        if(row.applied.stopReason!=DriveStopReason::NONE)legacy.reset();
        if(row.applied.stopReason!=DriveStopReason::NONE &&
           row.applied.stopReason!=DriveStopReason::REQUESTED_STOP)heading.reset();
        return row;
    }
};
struct Scenario {
    std::string name,category,assumption;
    std::vector<Input> inputs;
};
inline Scenario turn(float angle,float rate,float dwell,int direction,float acceleration=0) {
    Scenario s;
    s.category=acceleration>0?"accelerating_turn":"constant_rate_turn";
    s.name="turn_"+std::to_string((int)angle)+"deg_"+std::to_string((int)rate)+
        "dps_dwell"+std::to_string((int)(dwell*1000))+"_"+(direction>0?"cw":"ccw")+
        "_accel"+std::to_string((int)acceleration);
    s.assumption="Prescribed headings, not a hull model. Dwell may saturate steering before motion.";
    const float duration=dwell+(angle+20)/rate+(acceleration>0?rate/acceleration:0)+3;
    for(unsigned i=0;i<(unsigned)std::ceil(duration/.05f);++i) {
        const float t=i*.05f;
        float elapsed=std::fmax(0.0f,t-dwell),travel=rate*elapsed;
        if(acceleration>0) {
            const float ramp=rate/acceleration;
            travel=elapsed<ramp?.5f*acceleration*elapsed*elapsed:
                rate*(elapsed-.5f*ramp);
        }
        travel=std::fmin(travel,angle+20);
        Input in;in.heading=wrap(90-direction*angle+direction*travel);
        in.trueHeading=in.heading;s.inputs.push_back(in);
    }
    return s;
}
inline std::vector<Scenario> matrix() {
    std::vector<Scenario> all;
    for(float angle:{15.f,30.f,45.f,90.f,135.f,179.f})
        for(float rate:{1.f,2.f,5.f,10.f,15.f,20.f,30.f,45.f,60.f,90.f,120.f})
            for(float dwell:{0.f,1.5f})for(int direction:{-1,1})
                all.push_back(turn(angle,rate,dwell,direction));
    for(float rate:{15.f,30.f,60.f})for(float acceleration:{5.f,15.f,30.f,60.f})
        for(int direction:{-1,1})all.push_back(turn(90,rate,1.5f,direction,acceleration));
    for(float amplitude:{0.f,.2f,1.f,3.f,5.f})for(float period:{.2f,1.f,3.f}) {
        Scenario s{"heading_noise_"+std::to_string(amplitude)+"_period"+std::to_string(period),
            "heading_noise","Heading noise only; true heading remains90. Not physical yaw."};
        for(unsigned i=0;i<600;++i) {
            Input in;in.heading=wrap(90+amplitude*std::sin(i*.05f*6.2831853f/period));s.inputs.push_back(in);
        }all.push_back(s);
    }
    for(float amplitude:{.5f,1.f,3.f,5.f}) {
        Scenario s{"lateral_input_"+std::to_string(amplitude)+"m","lateral_LOS",
            "Prescribed lateral drift/GPS error through production6m LOS, no water model."};
        const Waypoint start=waypoint(0,0),end=waypoint(100,0);
        for(unsigned i=0;i<600;++i) {
            double lat,lon;Navigator::fromLocalEnu(20,99,i*.05*.7,
                amplitude*std::sin(i*.05*6.2831853/12),lat,lon);
            const auto los=Navigator::computeSegmentLos(lat,lon,start,end,
                GRID_LOS_LOOKAHEAD_M,WAYPOINT_RADIUS_M,LOS_PASS_MARGIN_M);
            Input in;in.target=los.bearingDeg;in.crossTrack=los.crossTrackM;
            in.distance=los.distToEndM;s.inputs.push_back(in);
        }all.push_back(s);
    }
    for(float angle:{5.f,15.f,30.f,60.f,90.f,180.f})for(bool reset:{false,true}) {
        Scenario s{"target_step_"+std::to_string((int)angle)+(reset?"_waypoint_reset":"_same_waypoint"),
            "target_change","Reset GRID heading only on waypoint change; final Drive is retained."};
        for(unsigned i=0;i<200;++i) {
            Input in;in.heading=in.trueHeading=0;in.target=i<30?90:wrap(90+angle);
            in.resetHeading=reset&&i==30;in.waypoint=i<30?0:(reset?1:0);s.inputs.push_back(in);
        }all.push_back(s);
    }
    for(const std::string kind:{"water_hold8s","compass_loss","gps_loss","estop",
                                "critical_battery","standby","rth_transition","spiral_transition",
                                "nan_heading","nan_target","loop_gap300ms","loop_jitter"}) {
        Scenario s{kind,"interlock_or_timing","Production controllers/gates with scripted task-level inputs."};
        for(unsigned i=0;i<300;++i) {
            Input in;const bool event=i>=40&&i<200;
            if(kind=="water_hold8s")in.hold=event;
            if(kind=="compass_loss")in.fresh=!event;
            if(kind=="gps_loss")in.gps=!event;
            if(kind=="estop")in.emergency=event;
            if(kind=="critical_battery"&&event)in.safety=SafetyState::CRITICAL_STOP;
            if(kind=="standby"&&event)in.state=STATE_STANDBY;
            if(kind=="rth_transition"&&event) {in.state=STATE_RTH;in.safety=SafetyState::LOW_BATTERY_RTH;}
            if(kind=="spiral_transition"&&event)in.state=STATE_SPIRAL_MAPPING;
            if(kind=="nan_heading"&&event)in.heading=NAN;
            if(kind=="nan_target"&&event)in.target=NAN;
            if(kind=="loop_gap300ms"&&i==40)in.dt=.3f;
            if(kind=="loop_jitter")in.dt=i%2?.025f:.075f;
            s.inputs.push_back(in);
        }all.push_back(s);
    }
    return all;
}
}
#endif
