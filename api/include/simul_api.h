#ifndef SIMUL_API_H
#define SIMUL_API_H

/*
 * Project Simul 의 C 인터페이스 (simul_core 공유 라이브러리: Windows simul_core.dll, Linux libsimul_core.so).
 * python 의 ctypes 로 바로 불러 쓸 수 있도록 C 타입만 쓴다 (python/simul/ 가 이 위에 만든 래퍼다).
 *
 * 약속
 *   - 길이는 m, 각도는 rad, 시간은 s, 자세는 쿼터니언 (x, y, z, w). 문자열은 UTF-8.
 *   - 한 simul_sim 은 한 스레드에서만 쓴다 (스레드 사이에서 같이 쓰면 안 된다).
 *   - 돌려주는 const char* 는 그 simul_sim 의 다음 API 호출 전까지만 유효하다. 바로 복사해서 쓸 것.
 *   - 실패하는 함수는 0 (또는 -1, NULL) 을 돌려주고 이유는 simul_last_error 로 읽는다. 성공하면 1 (또는 0 이상).
 *   - 명령(simul_set_pose, simul_move_to 등)은 큐에 들어가고 다음 simul_step 시작에 적용된다 (lockstep).
 *     한 번의 simul_step 안에서는 읽는 값이 모두 같은 시각의 것이다.
 */

#if defined(_WIN32)
#ifdef SIMUL_API_BUILD
#define SIMUL_API __declspec(dllexport)
#else
#define SIMUL_API __declspec(dllimport)
#endif
#else
#define SIMUL_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct simul_sim simul_sim;

#define SIMUL_API_VERSION 1

SIMUL_API int simul_api_version(void);          /* 이 라이브러리의 인터페이스 버전 (python 래퍼가 맞는지 확인한다) */
SIMUL_API const char* simul_version(void);

SIMUL_API simul_sim* simul_create(void);
SIMUL_API void simul_destroy(simul_sim* sim);
SIMUL_API const char* simul_last_error(simul_sim* sim);

/* ---- 모델 (오브젝트) ---- */
/* 모델 파일(.urdf, 또는 .stl .obj .dae .glb .gltf)을 올린다. 위치 x y z (m), 자세 roll pitch yaw (rad).
 * 성공하면 오브젝트 번호(0..)를, 실패하면 -1 을 돌려준다. 오브젝트 이름은 파일 이름에서 만들고 장면 안에서 겹치지 않는다. */
SIMUL_API int simul_add_model(simul_sim* sim, const char* path, double x, double y, double z, double roll, double pitch, double yaw);
SIMUL_API int simul_remove_model(simul_sim* sim, const char* name);
SIMUL_API void simul_clear(simul_sim* sim);                       /* 모델을 모두 내리고 시계를 0 으로 */
SIMUL_API void simul_add_search_dir(simul_sim* sim, const char* dir);   /* mesh / package:// 경로를 찾을 추가 폴더 */
SIMUL_API int simul_search_dir_count(simul_sim* sim);
SIMUL_API const char* simul_search_dir(simul_sim* sim, int i);
SIMUL_API int simul_warning_count(simul_sim* sim);                /* 모델을 올릴 때 나온 경고 (읽지 못한 메시 등) */
SIMUL_API const char* simul_warning(simul_sim* sim, int i);

/* ---- 오브젝트 정보 (읽기) ---- */
SIMUL_API int simul_object_count(simul_sim* sim);
SIMUL_API int simul_find_object(simul_sim* sim, const char* name);     /* 번호, 없으면 -1 */
SIMUL_API const char* simul_object_name(simul_sim* sim, int index);
SIMUL_API const char* simul_object_kind(simul_sim* sim, int index);    /* "robot" 또는 "model" */
SIMUL_API const char* simul_object_path(simul_sim* sim, int index);    /* 올린 파일 */
SIMUL_API int simul_object_pose(simul_sim* sim, int index, double out[7]);   /* x y z qx qy qz qw (월드 기준) */
SIMUL_API int simul_joint_count(simul_sim* sim, int index);            /* 움직이는 관절 수 */
SIMUL_API const char* simul_joint_name(simul_sim* sim, int index, int joint);
/* out: value, lower, upper, velocity(최대 속도, 0 이면 없음), has_limit(0/1), kind(1 회전, 2 직선, 3 연속 회전) */
SIMUL_API int simul_joint_info(simul_sim* sim, int index, int joint, double out[6]);

/* ---- 충돌 (지금 겹친 링크 쌍) ---- */
SIMUL_API int simul_collision_count(simul_sim* sim);               /* 부를 때마다 새로 계산한다 */
SIMUL_API const char* simul_collision_object_a(simul_sim* sim, int i);
SIMUL_API const char* simul_collision_link_a(simul_sim* sim, int i);
SIMUL_API const char* simul_collision_object_b(simul_sim* sim, int i);
SIMUL_API const char* simul_collision_link_b(simul_sim* sim, int i);
SIMUL_API int simul_collision_same_object(simul_sim* sim, int i);  /* 같은 오브젝트 안의 링크끼리 (자기충돌) 면 1 */
/* 위 문자열들은 다음 simul_collision_count 를 부르기 전까지 유효하다 */

/* ---- 명령 (큐에 넣는다. 적용은 다음 simul_step 시작) ---- */
SIMUL_API int simul_set_pose(simul_sim* sim, const char* name, double x, double y, double z, double qx, double qy, double qz, double qw);
SIMUL_API int simul_set_joint(simul_sim* sim, const char* name, const char* joint, double value);
SIMUL_API int simul_move_to(simul_sim* sim, const char* name, double x, double y, double z, double speed);                /* speed m/s */
SIMUL_API int simul_move_joint(simul_sim* sim, const char* name, const char* joint, double target, double speed);         /* speed <= 0 이면 관절의 최대 속도 */
SIMUL_API int simul_stop_motion(simul_sim* sim, const char* name);
SIMUL_API int simul_moving(simul_sim* sim, const char* name);                                                              /* move_to/move_joint 진행 중이면 1 */

/* ---- 시간 ---- */
SIMUL_API int simul_step(simul_sim* sim, double dt);               /* dt(s) 만큼 진행. 처음 부르면 시작한다 */
SIMUL_API double simul_time(simul_sim* sim);
SIMUL_API unsigned long long simul_tick_count(simul_sim* sim);
SIMUL_API void simul_reset_time(simul_sim* sim);                   /* 시계 0, 진행 중인 움직임과 명령을 비운다 (모델은 그대로) */

/* ---- 이벤트 (simul_step 이 쌓은 것을 꺼낸다) ---- */
/* type: 0 added, 1 removed, 2 arrived, 3 joint_reached, 4 command_failed */
SIMUL_API int simul_take_events(simul_sim* sim);                   /* 쌓인 이벤트를 꺼내 개수를 돌려준다 (이전에 꺼낸 것은 사라진다) */
SIMUL_API int simul_event_type(simul_sim* sim, int i);
SIMUL_API const char* simul_event_object(simul_sim* sim, int i);
SIMUL_API const char* simul_event_joint(simul_sim* sim, int i);
SIMUL_API const char* simul_event_message(simul_sim* sim, int i);
SIMUL_API double simul_event_time(simul_sim* sim, int i);

#ifdef __cplusplus
}
#endif

#endif /* SIMUL_API_H */
