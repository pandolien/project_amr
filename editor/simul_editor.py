#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Project Simul Editor (python + PyQt5).

프로젝트 폴더(main.py, objects/*.py, models/...)를 열어 python 코드를 고치고, 왼쪽 솔루션 탭에서 오브젝트와 관절을 보고,
오브젝트의 관절 슬라이더로 URDF 를 움직여 보고(왼쪽 위 3D 화면), 그 값을 코드로 넣고, F5 로 main.py 를 돌린다 (시뮬레이션은 simul 라이브러리가 DLL 로 돌린다).

실행:  python3 editor/simul_editor.py [프로젝트 폴더]       (필요: PyQt5, 새 프로젝트 만들기에는 빌드한 DLL: 환경 변수 SIMUL_LIB)
"""
import code
import contextlib
import io
import keyword
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "python"))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from PyQt5.QtCore import QTimer, QProcess, QProcessEnvironment, QRegExp, Qt, QDir
from PyQt5.QtGui import QColor, QFont, QKeySequence, QSyntaxHighlighter, QTextCharFormat
from PyQt5.QtWidgets import (QAction, QApplication, QDockWidget, QFileDialog, QFileSystemModel, QInputDialog, QMainWindow,
                             QDoubleSpinBox, QLineEdit, QMessageBox, QPlainTextEdit, QSlider, QTabWidget, QTreeView, QTreeWidget, QTreeWidgetItem,
                             QWidget, QHBoxLayout, QLabel, QSplitter, QVBoxLayout)


class PyHighlighter(QSyntaxHighlighter):
    KEYWORDS = ("and as assert break class continue def del elif else except finally for from global if import in is lambda "
                "nonlocal not or pass raise return try while with yield None True False self").split()

    def __init__(self, doc):
        QSyntaxHighlighter.__init__(self, doc)

        def fmt(color, bold=False):
            f = QTextCharFormat()
            f.setForeground(QColor(color))
            if bold:
                f.setFontWeight(QFont.Bold)
            return f
        self.rules = [(QRegExp(r"\b(%s)\b" % "|".join(self.KEYWORDS)), fmt("#0000c0", True)),
                      (QRegExp(r"\b[0-9]+(\.[0-9]+)?\b"), fmt("#a05000")),
                      (QRegExp(r"\"[^\"\\n]*\"|'[^'\\n]*'"), fmt("#007000")),
                      (QRegExp(r"#[^\n]*"), fmt("#808080"))]

    def highlightBlock(self, text):
        for rx, f in self.rules:
            i = rx.indexIn(text)
            while i >= 0:
                n = rx.matchedLength()
                self.setFormat(i, n, f)
                i = rx.indexIn(text, i + n)


class CodeEdit(QPlainTextEdit):
    def __init__(self, path):
        QPlainTextEdit.__init__(self)
        self.path = path
        f = QFont("monospace")
        f.setStyleHint(QFont.Monospace)
        self.setFont(f)
        self.setTabStopWidth(4 * self.fontMetrics().width(" "))
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        with open(path, encoding="utf-8") as fh:
            self.setPlainText(fh.read())
        self.document().setModified(False)
        self.hl = PyHighlighter(self.document())

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key_Return, Qt.Key_Enter):      # 자동 들여쓰기
            line = self.textCursor().block().text()
            indent = re.match(r"\s*", line).group(0)
            if line.rstrip().endswith(":"):
                indent += "    "
            QPlainTextEdit.keyPressEvent(self, e)
            self.insertPlainText(indent)
            return
        if e.key() == Qt.Key_Tab:
            self.insertPlainText("    ")
            return
        QPlainTextEdit.keyPressEvent(self, e)

    def save(self):
        with open(self.path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(self.toPlainText())
        self.document().setModified(False)


class PyConsole(QWidget):
    """Python Command: 장면에 직접 명령을 내리는 대화형 python.  sim, 오브젝트 이름(arm 등), step(), simul 을 쓸 수 있다."""

    def __init__(self):
        QWidget.__init__(self)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(5000)
        f = QFont("monospace")
        f.setStyleHint(QFont.Monospace)
        self.out.setFont(f)
        self.inp = QLineEdit()
        self.inp.setFont(f)
        self.inp.setPlaceholderText('예)  arm.set_joint("base_yaw", 1.0)     arm.joints["base_yaw"].value     step(100)')
        self.inp.returnPressed.connect(self.submit)
        lay.addWidget(self.out, 1)
        lay.addWidget(self.inp)
        self.history, self.hpos = [], 0
        self.ic = code.InteractiveConsole({})
        self.more = False

    def set_scene(self, sim):
        ns = self.ic.locals
        for k in [k for k, v in ns.items() if getattr(v, "_simul_obj", False)]:
            del ns[k]
        import simul
        ns.update({"simul": simul, "sim": sim})
        if sim is not None:
            def step(n=1, dt=0.01):
                for _ in range(int(n)):
                    sim.step(dt)
                return sim.time
            ns["step"] = step
            for name in sim.objects:
                if name.isidentifier() and not keyword.iskeyword(name):
                    ns[name] = sim.objects[name]
                    try:
                        ns[name]._simul_obj = True
                    except Exception:
                        pass
            self.out.appendPlainText("# sim, step(n, dt), 오브젝트: %s" % ", ".join(sorted(sim.objects)))

    def submit(self):
        line = self.inp.text()
        self.inp.clear()
        if line.strip():
            self.history.append(line)
        self.hpos = len(self.history)
        self.out.appendPlainText(("... " if self.more else ">>> ") + line)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            self.more = self.ic.push(line)
        text = buf.getvalue().rstrip("\n")
        if text:
            self.out.appendPlainText(text)

    def keyPressEvent(self, e):
        if self.inp.hasFocus() and e.key() in (Qt.Key_Up, Qt.Key_Down) and self.history:
            self.hpos = max(0, min(len(self.history), self.hpos + (-1 if e.key() == Qt.Key_Up else 1)))
            self.inp.setText(self.history[self.hpos] if self.hpos < len(self.history) else "")
            return
        QWidget.keyPressEvent(self, e)


class Editor(QMainWindow):
    def __init__(self, project=None):
        QMainWindow.__init__(self)
        self.setWindowTitle("Simul Editor")
        self.resize(1280, 800)
        self.project = None
        self.proc = None
        self.sim = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.tabs = QTabWidget()
        self.tabs.setTabsClosable(True)
        self.tabs.tabCloseRequested.connect(self.close_tab)
        self.view_holder = QWidget()
        self.view_layout = QVBoxLayout(self.view_holder)
        self.view_layout.setContentsMargins(0, 0, 0, 0)
        self.view_layout.addWidget(QLabel("3D View: 프로젝트를 열면 여기에 나옵니다"))

        # 솔루션 탭 (왼쪽): 프로젝트 파일 + 오브젝트/관절
        self.fs = QFileSystemModel()
        self.tree = QTreeView()
        self.tree.setModel(self.fs)
        for c in (1, 2, 3):
            self.tree.hideColumn(c)
        self.tree.setHeaderHidden(True)
        self.tree.doubleClicked.connect(lambda i: self.open_file(self.fs.filePath(i)))
        self.objects = QTreeWidget()
        self.objects.setHeaderLabels(["오브젝트 / 관절", "값 (슬라이더로 움직임)", "한계"])
        self.objects.itemDoubleClicked.connect(self.insert_name)
        sol = QSplitter(Qt.Vertical)
        sol.addWidget(self.labeled("솔루션 (프로젝트 파일)", self.tree))
        sol.addWidget(self.labeled("오브젝트 / 관절 (더블클릭: 이름 넣기)", self.objects))
        sol.setSizes([300, 480])

        # 가운데: 3D View / Python Command
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(5000)
        self.console = PyConsole()
        self.cmdtabs = QTabWidget()
        self.cmdtabs.addTab(self.console, "Python Command")
        self.cmdtabs.addTab(self.log, "실행 결과")
        mid = QSplitter(Qt.Vertical)
        mid.addWidget(self.view_holder)
        mid.addWidget(self.cmdtabs)
        mid.setSizes([400, 380])

        # 오른쪽: Editor
        main = QSplitter(Qt.Horizontal)
        main.addWidget(sol)
        main.addWidget(mid)
        main.addWidget(self.tabs)
        main.setSizes([400, 450, 430])
        self.setCentralWidget(main)

        m = self.menuBar().addMenu("파일")
        self.act(m, "새 프로젝트...", self.new_project, "Ctrl+N")
        self.act(m, "프로젝트 열기...", lambda: self.open_project(None), "Ctrl+O")
        self.act(m, "저장", self.save_current, "Ctrl+S")
        self.act(m, "모두 저장", self.save_all, "Ctrl+Shift+S")
        m = self.menuBar().addMenu("실행")
        self.act(m, "main.py 실행", self.run_project, "F5")
        self.act(m, "정지", self.stop_project, "Shift+F5")
        self.act(m, "오브젝트 정보 새로고침", self.refresh_objects, "F6")
        m = self.menuBar().addMenu("보기")
        self.act(m, "3D 화면 전체 보기", self.show_3d, "F3")
        self.act(m, "관절을 0 으로", self.zero_joints)
        self.act(m, "지금 관절 값을 코드로 넣기", self.insert_pose, "Ctrl+Shift+J")
        if project:
            self.open_project(project)

    def labeled(self, title, w):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        head = QLabel(" " + title)
        head.setStyleSheet("background:#dde3ea; padding:2px;")
        lay.addWidget(head)
        lay.addWidget(w, 1)
        return box

    def act(self, menu, text, fn, key=None):
        a = QAction(text, self)
        if key:
            a.setShortcut(QKeySequence(key))
        a.triggered.connect(lambda *_: fn())
        menu.addAction(a)
        return a

    # ---- 프로젝트 ----
    def new_project(self):
        urdfs, _ = QFileDialog.getOpenFileNames(self, "프로젝트에 넣을 URDF (여러 개 가능)", os.path.join(ROOT, "Data", "Object"), "URDF (*.urdf)")
        if not urdfs:
            return
        dest = QFileDialog.getExistingDirectory(self, "프로젝트를 만들 폴더(비어 있거나 새 폴더)")
        if not dest:
            return
        name, ok = QInputDialog.getText(self, "새 프로젝트", "프로젝트 이름", text="my_project")
        if not ok or not name:
            return
        lib = os.environ.get("SIMUL_LIB") or os.path.join(ROOT, "build", "lib")
        import new_project
        path = os.path.join(dest, name)
        lines = []
        try:
            rc = new_project.create_project(path, urdfs, lib, log=lines.append)
        except Exception as e:      # 모델을 못 읽는 경우 등
            rc, lines = 1, lines + [str(e)]
        self.log.appendPlainText("\n".join(lines))
        if rc == 0:
            self.open_project(path)
        else:
            QMessageBox.warning(self, "새 프로젝트", "만들지 못했습니다:\n" + "\n".join(lines[-3:]))

    def open_project(self, path):
        if path is None:
            path = QFileDialog.getExistingDirectory(self, "프로젝트 폴더 (main.py 가 있는 폴더)")
        if not path:
            return
        path = os.path.abspath(path)
        if not os.path.isfile(os.path.join(path, "main.py")):
            QMessageBox.warning(self, "프로젝트 열기", "main.py 가 없는 폴더입니다: " + path)
            return
        if not self.close_all_tabs():
            return
        self.project = path
        self.setWindowTitle("Simul Editor - " + path)
        self.fs.setRootPath(path)
        self.tree.setRootIndex(self.fs.index(path))
        self.fs.setFilter(QDir.AllDirs | QDir.Files | QDir.NoDotAndDotDot)
        self.open_file(os.path.join(path, "main.py"))
        self.refresh_objects()

    def refresh_objects(self):
        """models/ 의 URDF 를 DLL(simul_core)로 읽어 장면을 만들고, 관절마다 슬라이더를 단다 (움직여 보는 장면)."""
        self.objects.clear()
        self.timer.stop()
        if self.sim is not None:
            self.sim.close()
            self.sim = None
        if not self.project:
            return
        bundled = os.path.join(self.project, "simul")
        if os.path.isdir(bundled) and "SIMUL_LIB" not in os.environ:
            os.environ["SIMUL_LIB"] = bundled
        try:
            import simul
            self.sim = sim = simul.Simulation()
            for dp, _, fs in os.walk(os.path.join(self.project, "models")):
                for fn in sorted(fs):
                    if fn.endswith(".urdf"):
                        try:
                            sim.add_model(os.path.join(dp, fn))
                        except Exception as e:
                            self.log.appendPlainText("[%s] %s" % (fn, e))
            for name in sorted(sim.objects):
                o = sim.objects[name]
                top = QTreeWidgetItem([name, "", ""])
                top.setData(0, Qt.UserRole, name)
                self.objects.addTopLevelItem(top)
                for jn in sorted(o.joints):
                    j = o.joints[jn]
                    lim = "[%.3f, %.3f]" % (j.lower, j.upper) if getattr(j, "has_limit", True) else ""
                    c = QTreeWidgetItem([jn, "", lim])
                    c.setData(0, Qt.UserRole, jn)
                    top.addChild(c)
                    self.objects.setItemWidget(c, 1, self.joint_widget(name, jn, j))
            self.objects.expandAll()
            for c in (0, 2):
                self.objects.resizeColumnToContents(c)
            self.objects.setColumnWidth(1, 220)
            self.timer.start(30)
            self.embed_3d()
            self.console.set_scene(sim)
        except Exception as e:
            self.log.appendPlainText("오브젝트 정보를 읽지 못했습니다: %s" % e)

    def embed_3d(self):
        """3D 화면을 에디터 안(왼쪽 위)에 넣는다. simul_gui 와 같은 Qt 의 PyQt5 가 필요하다."""
        while self.view_layout.count():
            it = self.view_layout.takeAt(0)
            if it.widget() is not None:
                it.widget().setParent(None)
        try:
            self.view3d = self.sim.embed_widget()
            self.view_layout.addWidget(self.view3d)
            self.view3d.show()
            self.sim.fit_view()
        except Exception as e:
            self.view_layout.addWidget(QLabel("3D 화면을 만들 수 없습니다 (simul_gui 와 Qt 5.12 PyQt5 필요):\n%s" % e))

    def joint_widget(self, obj, jn, j):
        lo, hi = j.lower, j.upper
        if not getattr(j, "has_limit", True) or not hi > lo:
            lo, hi = -3.1416, 3.1416
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        sl = QSlider(Qt.Horizontal)
        sl.setRange(0, 1000)
        sp = QDoubleSpinBox()
        sp.setDecimals(4)
        sp.setRange(lo, hi)
        sp.setSingleStep(0.05)
        sp.setValue(j.value)
        sl.setValue(int(round((j.value - lo) / (hi - lo) * 1000)))
        lay.addWidget(sl, 1)
        lay.addWidget(sp)

        def apply(v):
            try:
                self.sim.objects[obj].set_joint(jn, v)
            except Exception as e:
                self.log.appendPlainText(str(e))

        def from_slider(x):
            v = lo + (hi - lo) * x / 1000.0
            sp.blockSignals(True)
            sp.setValue(v)
            sp.blockSignals(False)
            apply(sp.value())

        def from_spin(v):
            sl.blockSignals(True)
            sl.setValue(int(round((v - lo) / (hi - lo) * 1000)))
            sl.blockSignals(False)
            apply(v)
        sl.valueChanged.connect(from_slider)
        sp.valueChanged.connect(from_spin)
        w._spin, w._slider = sp, sl
        return w

    def tick(self):
        """슬라이더로 정한 값을 장면에 반영하고 3D 창을 갱신한다 (창이 닫혀도 장면은 남는다)."""
        if self.sim is None:
            return
        try:
            self.sim.step(0.001)
            if self.sim.window_open:
                self.sim.update_window()
        except Exception as e:
            self.timer.stop()
            self.log.appendPlainText("시뮬레이션 오류: %s" % e)

    def show_3d(self):
        """카메라를 모델 전체에 맞춘다 (3D 화면은 에디터 안에 있다)."""
        if self.sim is not None:
            self.sim.fit_view()

    def zero_joints(self):
        if self.sim is None:
            return
        for i in range(self.objects.topLevelItemCount()):
            top = self.objects.topLevelItem(i)
            for k in range(top.childCount()):
                w = self.objects.itemWidget(top.child(k), 1)
                w._spin.setValue(min(max(0.0, w._spin.minimum()), w._spin.maximum()))

    def insert_pose(self):
        """지금 슬라이더 값을 {"joints": {...}} 코드로 현재 편집 중인 파일에 넣는다."""
        ed = self.tabs.currentWidget()
        if ed is None or self.sim is None:
            return
        parts = []
        for i in range(self.objects.topLevelItemCount()):
            top = self.objects.topLevelItem(i)
            for k in range(top.childCount()):
                c = top.child(k)
                parts.append('"%s": %.4f' % (c.text(0), self.objects.itemWidget(c, 1)._spin.value()))
        ed.insertPlainText('{"joints": {%s}}' % ", ".join(parts))

    def insert_name(self, item, _col):
        ed = self.tabs.currentWidget()
        if ed is not None:
            ed.insertPlainText('"%s"' % item.data(0, Qt.UserRole))

    # ---- 파일 ----
    def open_file(self, path):
        if not os.path.isfile(path) or not path.endswith((".py", ".txt", ".md", ".urdf", ".xacro")):
            return
        for i in range(self.tabs.count()):
            if self.tabs.widget(i).path == path:
                self.tabs.setCurrentIndex(i)
                return
        ed = CodeEdit(path)
        ed.document().modificationChanged.connect(lambda _m, e=ed: self.mark(e))
        i = self.tabs.addTab(ed, os.path.basename(path))
        self.tabs.setTabToolTip(i, path)
        self.tabs.setCurrentIndex(i)

    def mark(self, ed):
        i = self.tabs.indexOf(ed)
        if i >= 0:
            self.tabs.setTabText(i, os.path.basename(ed.path) + (" *" if ed.document().isModified() else ""))

    def save_current(self):
        ed = self.tabs.currentWidget()
        if ed is not None:
            ed.save()

    def save_all(self):
        for i in range(self.tabs.count()):
            self.tabs.widget(i).save()

    def close_tab(self, i):
        ed = self.tabs.widget(i)
        if ed.document().isModified():
            r = QMessageBox.question(self, "저장", "%s 을(를) 저장할까요?" % os.path.basename(ed.path), QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            if r == QMessageBox.Cancel:
                return False
            if r == QMessageBox.Yes:
                ed.save()
        self.tabs.removeTab(i)
        return True

    def close_all_tabs(self):
        while self.tabs.count():
            if self.close_tab(0) is False:
                return False
        return True

    def closeEvent(self, e):
        self.stop_project()
        self.timer.stop()
        if self.sim is not None:
            self.sim.close()
            self.sim = None
        e.accept() if self.close_all_tabs() else e.ignore()

    # ---- 실행 ----
    def python_for_project(self):
        for rel in (".venv/bin/python", ".venv/Scripts/python.exe"):
            p = os.path.join(self.project, rel)
            if os.path.isfile(p):
                return p
        return sys.executable

    def run_project(self):
        if not self.project:
            return
        self.stop_project()
        self.save_all()
        self.proc = QProcess(self)
        self.proc.setWorkingDirectory(self.project)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        env = QProcessEnvironment.systemEnvironment()
        env.insert("PYTHONUNBUFFERED", "1")
        self.proc.setProcessEnvironment(env)
        self.proc.readyRead.connect(lambda: self.log.appendPlainText(bytes(self.proc.readAll()).decode("utf-8", "replace").rstrip("\n")))
        self.proc.finished.connect(lambda code, _st: self.log.appendPlainText("--- 끝 (종료 코드 %d) ---" % code))
        self.log.appendPlainText("--- main.py 실행 (%s) ---" % self.python_for_project())
        self.proc.start(self.python_for_project(), ["main.py"])

    def stop_project(self):
        if self.proc is not None and self.proc.state() != QProcess.NotRunning:
            self.proc.kill()
            self.proc.waitForFinished(2000)


def main():
    app = QApplication(sys.argv)
    w = Editor(sys.argv[1] if len(sys.argv) > 1 else None)
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
