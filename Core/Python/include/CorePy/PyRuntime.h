// 앱 안에 내장한 Python 인터프리터와 `simul` 모듈 (저수준 다리). Engine(CSimul) 위에서 동작한다.
//
//   구조:  Simul(C++ 엔진)  ->  Python 인터프리터  ->  Main.py (모든 오브젝트 데이터를 쥐고 엔진과 통신)  ->  등록된 외부 파일의 클래스
//   이 클래스는 앞의 두 단계다: 인터프리터를 띄우고, 엔진의 오브젝트 정보와 명령을 `simul` 모듈로 내놓고, 매 틱/이벤트를 Python 쪽 콜백으로 전달한다.
//   Main.py (Data/Python/main.py) 가 이 위에서 오브젝트 데이터 보관, 외부 클래스 등록/실행, 오브젝트별 명령 권한을 맡는다.
//
//   Python 은 선택 사항이다: SIMUL_BUILD_PYTHON=OFF 이거나 개발 헤더가 없으면 이 라이브러리는 만들어지지 않는다.
//   인터프리터는 프로세스에 하나뿐이라 Runtime 도 하나만 만든다.
#ifndef COREPY_PYRUNTIME_H
#define COREPY_PYRUNTIME_H

#include "CBase/String.h"
#include "CSimul/Engine.h"

namespace simul {
namespace py {

// print / 오류 출력을 받는다. isError 는 stderr (예외 traceback 포함). 한 줄씩 (끝의 줄바꿈 없이) 불린다.
typedef void (*OutputFn)(bool isError, const char* line, void* user);

class Runtime : public sim::Listener {
public:
    explicit Runtime(sim::Engine* engine);
    ~Runtime();

    // 인터프리터를 띄우고 `simul` 모듈을 만든다. 실패하면 false 와 이유
    bool Initialize(String* error);
    bool Initialized() const { return initialized_; }
    void SetOutput(OutputFn fn, void* user) { out_ = fn; outUser_ = user; }
    static String PythonVersion();   // 링크된 Python 버전 ("3.8.10" 등)

    // 스크립트를 실행한다 (__main__ 이름공간. 그 폴더가 sys.path 앞에 들어간다). 예외가 나면 traceback 을 출력으로 보내고 false.
    bool RunFile(const String& path, String* error);
    // 한 줄 또는 여러 줄 코드를 대화형처럼 실행한다 (식의 값은 출력으로 나온다)
    bool RunSnippet(const String& code, String* error);

    // 스크립트가 simul.quit() 나 SystemExit 로 종료를 요청했는가
    bool QuitRequested() const { return quit_; }
    void ClearQuit() { quit_ = false; }

    // sim::Listener: 엔진의 시작/틱/이벤트/정지를 Python 콜백(simul.on_start 등)으로 넘긴다
    void OnStart(sim::Engine&) override;
    void OnTick(sim::Engine&, double time, double dt) override;
    void OnEvent(sim::Engine&, const sim::Event&) override;
    void OnStop(sim::Engine&) override;

    // 내부용 (simul 모듈이 쓴다)
    sim::Engine* engine() const { return engine_; }
    void RequestQuit() { quit_ = true; }
    void Emit(bool isError, const char* text);
    void FlushOutput();

private:
    struct Impl;
    void callAll(int kind, const char* what, const sim::Event* ev, double t, double dt);
    bool report(String* error);

    sim::Engine* engine_;
    Impl* impl_;
    bool initialized_;
    bool quit_;
    OutputFn out_;
    void* outUser_;
};

}  // namespace py
}  // namespace simul

#endif  // COREPY_PYRUNTIME_H
