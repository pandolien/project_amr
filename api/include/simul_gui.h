#ifndef SIMUL_GUI_H
#define SIMUL_GUI_H

/*
 * 3D 창 (simul_gui 공유 라이브러리, Qt 필요). simul_core 의 시뮬레이션을 화면에 보여 주는 보기 전용 창이다.
 * 시뮬레이션의 진실은 simul_core 에 있고, 창은 simul_window_update 를 부를 때마다 거기서 오브젝트 자세와 관절 값을 읽어 그린다.
 * 창이 필요 없으면 이 라이브러리를 부르지 않으면 된다 (Qt 없이 simul_core 만 쓴다).
 *
 * 창은 simul_window_update 를 부르는 스레드(보통 python 의 메인 스레드)에서만 다룬다. 이벤트(마우스, 닫기 등)도
 * 그 호출 안에서 처리되므로, 시뮬레이션 루프에서 주기적으로 불러 주어야 창이 반응한다.
 */
#include "simul_api.h"

#if defined(_WIN32)
#ifdef SIMUL_GUI_BUILD
#define SIMUL_GUI_API __declspec(dllexport)
#else
#define SIMUL_GUI_API __declspec(dllimport)
#endif
#else
#define SIMUL_GUI_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

SIMUL_GUI_API int simul_gui_api_version(void);
/* 창을 열고 보여 준다. 이미 열려 있으면 1. 열지 못하면 0 (이유는 simul_gui_last_error). */
SIMUL_GUI_API int simul_window_open(simul_sim* sim, const char* title, int width, int height);
/* 장면을 맞추고(모델이 바뀌었으면 다시 올림) 창의 이벤트를 처리한다. 창이 열려 있으면 1, 닫혔으면 0. */
/* 별도 창을 띄우지 않고 3D 위젯만 만든다 (호출한 Qt 프로그램이 자기 창 안에 넣는다).
 * simul_window_widget 은 QWidget* 를 돌려주고, 부모를 정해 주면 Qt 가 소유한다 (그 뒤 simul_window_close 는 안전하다). */
SIMUL_GUI_API int simul_window_create_embedded(simul_sim* sim);
SIMUL_GUI_API void* simul_window_widget(simul_sim* sim);
SIMUL_GUI_API int simul_window_update(simul_sim* sim);
SIMUL_GUI_API int simul_window_is_open(simul_sim* sim);
SIMUL_GUI_API void simul_window_close(simul_sim* sim);
/* 지금 화면을 PNG 등 이미지 파일로 저장한다 (확장자로 결정). 성공하면 1. */
SIMUL_GUI_API int simul_window_save_image(simul_sim* sim, const char* path);
SIMUL_GUI_API int simul_window_fit_view(simul_sim* sim);           /* 모델 전체가 보이게 카메라를 맞춘다 */
SIMUL_GUI_API const char* simul_gui_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* SIMUL_GUI_H */
