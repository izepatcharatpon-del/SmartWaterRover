#include <unity.h>
#include "ScenarioRig.h"
#include "core/WaterSamplePolicy.h"
#include <cstring>

void setUp() {}
void tearDown() {}
using namespace OpenWater;

void test_open_water_matrix_preserves_limits_deadband_and_neutral_reversal() {
    for(const auto& scenario:matrix()) {
        Rig rig;int oldL=0,oldR=0;
        for(const auto& input:scenario.inputs) {
            const auto r=rig.tick(input);
            for(int pwm:{r.applied.leftPwm,r.applied.rightPwm}) {
                TEST_ASSERT_TRUE(std::abs(pwm)<=MOTOR_PWM_LIMIT);
                TEST_ASSERT_TRUE(pwm==0||std::abs(pwm)>=MOTOR_MIN_PWM);
            }
            TEST_ASSERT_TRUE(oldL*r.applied.leftPwm>=0);
            TEST_ASSERT_TRUE(oldR*r.applied.rightPwm>=0);
            oldL=r.applied.leftPwm;oldR=r.applied.rightPwm;
        }
    }
}

void test_water_hold_has_160_zero_ticks_and_resumes_without_capture_latch() {
    Rig rig;Input in;
    for(int i=0;i<20;++i)TEST_ASSERT_TRUE(rig.tick(in).applied.leftPwm>0);
    in.hold=true;
    for(int i=0;i<160;++i) {
        const auto r=rig.tick(in);
        TEST_ASSERT_EQUAL_INT(0,r.applied.leftPwm);
        TEST_ASSERT_EQUAL_INT(0,r.applied.rightPwm);
        TEST_ASSERT_TRUE(r.applied.stopReason==DriveStopReason::WATER_SAMPLE);
    }
    in.hold=false;
    for(int i=0;i<400;++i) {
        const auto r=rig.tick(in);TEST_ASSERT_TRUE(r.applied.leftPwm>0&&r.applied.rightPwm>0);
    }
}

void test_all_final_drive_interlock_combinations_obey_stop_contract() {
    for(int state=0;state<5;++state)for(int safety=0;safety<5;++safety)
    for(int emergency=0;emergency<2;++emergency)for(int nav=0;nav<2;++nav)
    for(int hold=0;hold<2;++hold)for(int valid=0;valid<2;++valid)
    for(int source=0;source<5;++source) {
        DriveController d(MOTOR_PWM_LIMIT,MOTOR_MIN_PWM,MOTOR_PWM_SLEW_PER_SEC);
        DriveInterlocks locks{(BoatState)state,(SafetyState)safety,(bool)emergency,(bool)nav,(bool)hold};
        const auto out=d.resolve({(DriveSource)source,110,110,(bool)valid},locks,.05f);
        const bool fatal=emergency||safety>=2||state==STATE_EMERGENCY_STOP;
        const bool sourceOk=(state==1&&(source==1||source==2))||(state==2&&source==3)||(state==3&&source==4);
        const bool allowed=!fatal&&state!=0&&nav&&!hold&&valid&&sourceOk;
        TEST_ASSERT_EQUAL_INT(allowed,out.leftPwm!=0||out.rightPwm!=0);
        if(!allowed)TEST_ASSERT_EQUAL_INT(0,out.leftPwm+out.rightPwm);
    }
}

void test_fsm_navigation_loss_estop_battery_and_spiral_ownership() {
    FsmInputs in{};in.gpsOk=in.compassOk=in.hasWaypoints=in.homeSet=in.passedFirstWaypoint=true;
    in.safety=SafetyState::NORMAL;MissionFsm f;
    in.startCommand=true;TEST_ASSERT_EQUAL(STATE_GRID_PATROL,f.tick(in));in.startCommand=false;
    in.safety=SafetyState::LOW_BATTERY_RTH;TEST_ASSERT_EQUAL(STATE_RTH,f.tick(in));
    in.safety=SafetyState::CRITICAL_STOP;TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP,f.tick(in));
    in.safety=SafetyState::NORMAL;TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP,f.tick(in));
    in.clearEstop=true;TEST_ASSERT_EQUAL(STATE_STANDBY,f.tick(in));in.clearEstop=false;
    in.startCommand=true;TEST_ASSERT_EQUAL(STATE_GRID_PATROL,f.tick(in));in.startCommand=false;
    in.forceSpiralCommand=true;TEST_ASSERT_EQUAL(STATE_SPIRAL_MAPPING,f.tick(in));in.forceSpiralCommand=false;
    in.spiralComplete=true;TEST_ASSERT_EQUAL(STATE_GRID_PATROL,f.tick(in));in.spiralComplete=false;
    in.gpsFault=true;TEST_ASSERT_EQUAL(STATE_EMERGENCY_STOP,f.tick(in));
}

void test_actual_waypoint_advance_and_sample_boundary_use_production_geometry() {
    Navigator nav;WaypointArrayPacket p{};p.waypoint_count=3;p.packet_index=0;
    for(int i=0;i<3;++i)p.waypoints[i]=waypoint(i==0?30:33,i==2?30:0);
    TEST_ASSERT_TRUE(nav.loadWaypoints(p));nav.setMissionOrigin(20,99);
    double lat,lon;Navigator::fromLocalEnu(20,99,30,0,lat,lon);
    TEST_ASSERT_EQUAL_INT(1,nav.skipReached(lat,lon,GRID_LOS_LOOKAHEAD_M,WAYPOINT_RADIUS_M,LOS_PASS_MARGIN_M));
    TEST_ASSERT_EQUAL_INT(0,nav.skipReached(lat,lon,GRID_LOS_LOOKAHEAD_M,WAYPOINT_RADIUS_M,LOS_PASS_MARGIN_M));
    Navigator::fromLocalEnu(20,99,33,0,lat,lon);
    const int before=nav.index();TEST_ASSERT_EQUAL_INT(1,nav.skipReached(lat,lon,GRID_LOS_LOOKAHEAD_M,WAYPOINT_RADIUS_M,LOS_PASS_MARGIN_M));
    TEST_ASSERT_TRUE(WaterSamplePolicy::crossedBoundary(before,nav.index(),nav.count(),WATER_SAMPLE_EVERY_N_WAYPOINTS));
    TEST_ASSERT_FALSE(WaterSamplePolicy::missionCompleteForFsm(true,true));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_open_water_matrix_preserves_limits_deadband_and_neutral_reversal);
    RUN_TEST(test_water_hold_has_160_zero_ticks_and_resumes_without_capture_latch);
    RUN_TEST(test_all_final_drive_interlock_combinations_obey_stop_contract);
    RUN_TEST(test_fsm_navigation_loss_estop_battery_and_spiral_ownership);
    RUN_TEST(test_actual_waypoint_advance_and_sample_boundary_use_production_geometry);
    return UNITY_END();
}
