// simul_core 의 C 인터페이스 (api/include/simul_api.h) 시험
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "simul_api.h"

static int g_fail = 0;
#define CHECK(cond, msg)                         \
    do {                                         \
        if (!(cond)) {                           \
            printf("FAIL: %s\n", msg);           \
            g_fail++;                            \
        }                                        \
    } while (0)

int main() {
    const char* data = SIMUL_TEST_DATA_DIR;
    char arm[512], marker[512];
    snprintf(arm, sizeof(arm), "%s/Object/samples/arm.urdf", data);
    snprintf(marker, sizeof(marker), "%s/Object/markers/aruco_4x4_id0.urdf", data);

    CHECK(simul_api_version() == SIMUL_API_VERSION, "api version");
    simul_sim* s = simul_create();
    CHECK(s != NULL, "create");

    CHECK(simul_add_model(s, "does/not/exist.urdf", 0, 0, 0, 0, 0, 0) == -1, "missing file is rejected");
    CHECK(strlen(simul_last_error(s)) > 0, "error text is set");
    CHECK(simul_add_model(NULL, arm, 0, 0, 0, 0, 0, 0) == -1, "NULL simulation is rejected");
    CHECK(simul_add_model(s, arm, NAN, 0, 0, 0, 0, 0) == -1, "non finite position is rejected");

    const int a = simul_add_model(s, arm, 0.1, 0.2, 0.3, 0, 0, 1.5707963);
    CHECK(a == 0, "first model is object 0");
    CHECK(simul_object_count(s) == 1, "one object");
    CHECK(strcmp(simul_object_name(s, 0), "arm") == 0, "object name from file name");
    CHECK(strcmp(simul_object_kind(s, 0), "robot") == 0, "robot kind");
    CHECK(simul_find_object(s, "arm") == 0 && simul_find_object(s, "zzz") == -1, "find_object");

    double pose[7];
    CHECK(simul_object_pose(s, 0, pose) == 1, "pose");
    CHECK(fabs(pose[0] - 0.1) < 1e-5 && fabs(pose[1] - 0.2) < 1e-5 && fabs(pose[2] - 0.3) < 1e-5, "placement position");
    CHECK(fabs(pose[5] - sin(1.5707963 / 2)) < 1e-4 && fabs(pose[6] - cos(1.5707963 / 2)) < 1e-4, "yaw as quaternion");

    CHECK(simul_joint_count(s, 0) == 4, "joint count");
    double ji[6];
    CHECK(simul_joint_info(s, 0, 1, ji) == 1 && strcmp(simul_joint_name(s, 0, 1), "shoulder_pitch") == 0, "joint 1 is shoulder_pitch");
    CHECK(ji[4] == 1.0 && fabs(ji[2] - 1.57) < 1e-3, "joint limit is reported");

    CHECK(simul_move_joint(s, "arm", "shoulder_pitch", 0.5, 1.0) == 1, "move_joint accepted");
    CHECK(simul_move_joint(s, "arm", "shoulder_pitch", 99.0, 1.0) == 0 && strstr(simul_last_error(s), "한계") != NULL, "target beyond limit is rejected with the reason");
    CHECK(simul_move_joint(s, "nobody", "shoulder_pitch", 0.1, 1.0) == 0, "unknown object is rejected");
    CHECK(simul_move_joint(s, "arm", "shoulder_pitch", 0.4, 1.0) == 1, "a later command replaces the earlier motion");
    simul_joint_info(s, 0, 1, ji);
    CHECK(ji[0] == 0.0, "commands apply at the next step, not at once");
    for (int i = 0; i < 100; i++) CHECK(simul_step(s, 0.01) == 1, "step");
    simul_joint_info(s, 0, 1, ji);
    CHECK(fabs(ji[0] - 0.4) < 1e-5, "joint reached the target");
    CHECK(fabs(simul_time(s) - 1.0) < 1e-9 && simul_tick_count(s) == 100, "time and tick count");
    CHECK(simul_moving(s, "arm") == 0, "no longer moving");
    CHECK(simul_step(s, -1.0) == 0 && simul_step(s, NAN) == 0, "bad dt is rejected");

    int ev = simul_take_events(s);
    int added = 0, reached = 0;
    for (int i = 0; i < ev; i++) {
        added += simul_event_type(s, i) == 0;
        reached += simul_event_type(s, i) == 3 && strcmp(simul_event_joint(s, i), "shoulder_pitch") == 0;
    }
    CHECK(added == 1 && reached == 1, "added and joint_reached events");
    CHECK(simul_take_events(s) == 0, "events are taken only once");

    CHECK(simul_set_pose(s, "arm", 1, 0, 0, 0, 0, 0, 1) == 1, "set_pose accepted");
    CHECK(simul_set_joint(s, "arm", "elbow_pitch", 0.3) == 1, "set_joint accepted");
    simul_step(s, 0.01);
    simul_object_pose(s, 0, pose);
    simul_joint_info(s, 0, 2, ji);
    CHECK(fabs(pose[0] - 1.0) < 1e-5 && fabs(ji[0] - 0.3) < 1e-5, "set_pose and set_joint applied");

    // 충돌: 팔 옆에 마커를 둔다
    simul_set_pose(s, "arm", 0, 0, 0, 0, 0, 0, 1);
    simul_step(s, 0.01);
    CHECK(simul_collision_count(s) == 0, "nothing collides at first");
    const int m = simul_add_model(s, marker, 0.3, 0, 0.1, 0, 0, 0);
    CHECK(m == 1 && simul_object_count(s) == 2, "second model");
    const int n = simul_collision_count(s);
    int armVsMarker = 0;
    for (int i = 0; i < n; i++) {
        const char* oa = simul_collision_object_a(s, i);
        const char* ob = simul_collision_object_b(s, i);
        armVsMarker += (strcmp(oa, "arm") == 0 && strcmp(ob, "aruco_4x4_id0") == 0) || (strcmp(ob, "arm") == 0 && strcmp(oa, "aruco_4x4_id0") == 0);
        CHECK(simul_collision_same_object(s, i) == 0, "arm and marker are different objects");
    }
    CHECK(armVsMarker >= 1, "arm vs marker collision is reported");
    CHECK(simul_remove_model(s, "aruco_4x4_id0") == 1 && simul_object_count(s) == 1, "remove model");
    CHECK(simul_collision_count(s) == 0, "no collision after removing it");

    simul_clear(s);
    CHECK(simul_object_count(s) == 0 && simul_time(s) == 0.0, "clear");
    simul_destroy(s);
    if (g_fail) {
        printf("test_capi: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_capi: all passed\n");
    return 0;
}
