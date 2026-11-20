#ifndef CORECQT_PYTHONPANEL_H
#define CORECQT_PYTHONPANEL_H

#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QVector>
#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;

// 파이썬 코드 편집기: 줄 번호, 문법 색, 4칸 들여쓰기(Tab), 줄바꿈 때 자동 들여쓰기(':' 뒤에는 한 단계 더).
class PythonEditor : public QPlainTextEdit {
    Q_OBJECT
public:
    explicit PythonEditor(QWidget* parent = nullptr);
    int LineNumberWidth() const;
    void PaintLineNumbers(QPaintEvent* e);

protected:
    void resizeEvent(QResizeEvent* e) override;
    void keyPressEvent(QKeyEvent* e) override;

private slots:
    void updateMargin();
    void updateNumbers(const QRect& rect, int dy);
    void highlightLine();

private:
    QWidget* numbers_;
};

class PythonHighlighter : public QSyntaxHighlighter {
public:
    explicit PythonHighlighter(QTextDocument* doc);

protected:
    void highlightBlock(const QString& text) override;

private:
    struct Rule {
        QRegularExpression re;
        QTextCharFormat fmt;
    };
    QVector<Rule> rules_;
    QTextCharFormat string_, comment_;
};

// 오른쪽 도크에 넣는 "Python 편집기": 파일 열기/저장, 새 클래스 틀, 저장하고 등록, 한 줄 실행.
// 등록과 실행은 이 부품이 하지 않는다. 시그널을 받은 호스트가 한다 (Main.py 에 register_file 등).
class PythonPanel : public QWidget {
    Q_OBJECT
public:
    explicit PythonPanel(QWidget* parent = nullptr);

    void SetDefaultDir(const QString& dir) { defaultDir_ = dir; }
    bool Open(const QString& path);
    bool Save();                       // 이름이 없으면 저장 위치를 묻는다. 취소하거나 실패하면 false
    QString FilePath() const { return path_; }
    bool Modified() const { return editor_->document()->isModified(); }
    PythonEditor* editor() const { return editor_; }
    void NewFromTemplate();
    static QString Template();         // 새 파일에 들어가는 Behavior 틀

signals:
    void registerRequested(const QString& path, const QString& objects);   // 저장 후 등록 (objects: 쉼표로 나눈 권한, 비면 파일의 objects 설정)
    void snippetRequested(const QString& code);                           // 아래 입력줄 실행
    void fileChanged(const QString& path);

private slots:
    void onOpen();
    void onSaveAs();
    void onRegister();
    void onRunLine();
    void updateTitle();

private:
    bool saveTo(const QString& path);
    bool maybeDiscard();
    PythonEditor* editor_;
    PythonHighlighter* highlighter_;
    QLabel* title_;
    QLineEdit* objects_;
    QLineEdit* line_;
    QString path_;
    QString defaultDir_;
};

#endif  // CORECQT_PYTHONPANEL_H
