#include <unity.h>
#include "EmpiricalPlant.h"

void setUp() {}
void tearDown() {}

void test_yaw_inertia_keeps_turning_after_motor_cut_then_decays() {
    EmpiricalPlant p;p.rate=20;p.yawTau=2;p.velocityDelay=0;
    for(int i=0;i<40;++i)p.tick(0,0);
    TEST_ASSERT_FLOAT_WITHIN(.001,20/std::exp(1.0),p.rate);
    TEST_ASSERT_FLOAT_WITHIN(.001,90+40*(1-std::exp(-1.0)),p.heading);
}
void test_coast_velocity_and_distance_match_exponential_solution() {
    EmpiricalPlant p;p.ve=.5;p.velocityTau=8;p.velocityDelay=0;
    for(int i=0;i<160;++i)p.tick(0,0);
    TEST_ASSERT_FLOAT_WITHIN(.001,.5/std::exp(1.0),p.ve);
    TEST_ASSERT_FLOAT_WITHIN(.001,4*(1-std::exp(-1.0)),p.east);
}
void test_effective_drift_moves_gps_output_even_with_zero_motor_input() {
    EmpiricalPlant p;p.velocityDelay=0;p.driftEast=.03;p.ve=.03;
    p.driftNorth=-.02;p.vn=-.02;
    for(int i=0;i<200;++i)p.tick(0,0);
    TEST_ASSERT_FLOAT_WITHIN(.001,.3,p.east);
    TEST_ASSERT_FLOAT_WITHIN(.001,-.2,p.north);
}
void test_identified_observation_delay_is_not_an_instant_motor_effect() {
    EmpiricalPlant p;p.velocityDelay=1.5f;
    for(int i=0;i<30;++i)p.tick(110,110);
    TEST_ASSERT_FLOAT_WITHIN(.00001,0,p.east);
    for(int i=0;i<30;++i)p.tick(110,110);
    TEST_ASSERT_TRUE(p.east>0&&p.ve>0);
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_yaw_inertia_keeps_turning_after_motor_cut_then_decays);
    RUN_TEST(test_coast_velocity_and_distance_match_exponential_solution);
    RUN_TEST(test_effective_drift_moves_gps_output_even_with_zero_motor_input);
    RUN_TEST(test_identified_observation_delay_is_not_an_instant_motor_effect);
    return UNITY_END();
}
