#include "CoreQt/SunPanel.h"

#include <QCheckBox>
#include <QDateTimeEdit>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "CoreQt/ModelViewer.h"

namespace {

QString BearingName(double az) {
    static const char* names[8] = {"북", "북동", "동", "남동", "남", "남서", "서", "북서"};
    return QString::fromUtf8(names[static_cast<int>((az + 22.5) / 45.0) % 8]);
}

QString HHMM(double minute) {
    int m = static_cast<int>(minute + 0.5);
    m = ((m % 1440) + 1440) % 1440;
    return QString("%1:%2").arg(m / 60, 2, 10, QChar('0')).arg(m % 60, 2, 10, QChar('0'));
}

}  // namespace

SunPanel::SunPanel(ModelViewer* viewer, QWidget* parent) : QWidget(parent), viewer_(viewer), updating_(false) {
    QVBoxLayout* top = new QVBoxLayout(this);
    enabled_ = new QCheckBox(QString::fromUtf8("태양 조명 사용 (끄면 카메라에 고정된 기본 조명)"));
    top->addWidget(enabled_);
    shadows_ = new QCheckBox(QString::fromUtf8("그림자 (3D 화면, 태양 조명일 때)"));
    shadows_->setChecked(viewer_->ShadowsEnabled());
    top->addWidget(shadows_);
    connect(shadows_, &QCheckBox::toggled, viewer_, &ModelViewer::SetShadowsEnabled);
    QFormLayout* form = new QFormLayout;
    time_ = new QDateTimeEdit;
    time_->setDisplayFormat("yyyy-MM-dd HH:mm");
    time_->setCalendarPopup(true);
    time_->setDateRange(QDate(1800, 1, 1), QDate(2200, 12, 31));
    form->addRow(QString::fromUtf8("현지 날짜/시각"), time_);
    utc_ = new QDoubleSpinBox;
    utc_->setRange(-14.0, 14.0);
    utc_->setDecimals(2);
    utc_->setSingleStep(0.5);
    utc_->setPrefix("UTC ");
    form->addRow(QString::fromUtf8("시간대 (UTC 오프셋, 시간)"), utc_);
    lat_ = new QDoubleSpinBox;
    lat_->setRange(-90.0, 90.0);
    lat_->setDecimals(4);
    lat_->setSuffix(QString::fromUtf8(" °"));
    form->addRow(QString::fromUtf8("위도 (북 +, 남 -)"), lat_);
    lon_ = new QDoubleSpinBox;
    lon_->setRange(-180.0, 180.0);
    lon_->setDecimals(4);
    lon_->setSuffix(QString::fromUtf8(" °"));
    form->addRow(QString::fromUtf8("경도 (동 +, 서 -)"), lon_);
    north_ = new QDoubleSpinBox;
    north_->setRange(0.0, 359.99);
    north_->setDecimals(1);
    north_->setWrapping(true);
    north_->setSuffix(QString::fromUtf8(" °"));
    north_->setToolTip(QString::fromUtf8("월드 +y 축이 가리키는 방위 (북에서 시계 방향). 0 이면 +y 가 북, +x 가 동"));
    form->addRow(QString::fromUtf8("월드 +y 의 방위"), north_);
    top->addLayout(form);
    QPushButton* now = new QPushButton(QString::fromUtf8("지금 시각으로"));
    top->addWidget(now);
    lapse_ = new QCheckBox(QString::fromUtf8("시간 흐르게 하기"));
    top->addWidget(lapse_);
    QFormLayout* f2 = new QFormLayout;
    speed_ = new QDoubleSpinBox;
    speed_->setRange(0.1, 1440.0);
    speed_->setValue(30.0);
    speed_->setSuffix(QString::fromUtf8(" 분 / 초"));
    f2->addRow(QString::fromUtf8("속도"), speed_);
    top->addLayout(f2);
    info_ = new QLabel;
    info_->setWordWrap(true);
    info_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    top->addWidget(info_);
    top->addStretch(1);

    timer_ = new QTimer(this);
    timer_->setInterval(100);
    connect(timer_, &QTimer::timeout, this, &SunPanel::tick);

    connect(enabled_, &QCheckBox::toggled, this, &SunPanel::applyFromWidgets);
    connect(time_, &QDateTimeEdit::dateTimeChanged, this, &SunPanel::applyFromWidgets);
    connect(utc_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SunPanel::applyFromWidgets);
    connect(lat_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SunPanel::applyFromWidgets);
    connect(lon_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SunPanel::applyFromWidgets);
    connect(north_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &SunPanel::applyFromWidgets);
    connect(now, &QPushButton::clicked, this, &SunPanel::setNow);
    connect(lapse_, &QCheckBox::toggled, this, [this](bool on) {
        if (on) {
            clock_.start();
            timer_->start();
        } else {
            timer_->stop();
        }
    });
    connect(viewer_, &ModelViewer::sunChanged, this, &SunPanel::refreshFromViewer);
    refreshFromViewer();
}

bool SunPanel::TimeLapseRunning() const { return timer_->isActive(); }

void SunPanel::applyFromWidgets() {
    if (updating_) return;
    ModelViewer::SunSettings s;
    s.enabled = enabled_->isChecked();
    const QDateTime dt = time_->dateTime();
    s.time.year = dt.date().year();
    s.time.month = dt.date().month();
    s.time.day = dt.date().day();
    s.time.hour = dt.time().hour();
    s.time.minute = dt.time().minute();
    s.time.second = 0.0;
    s.time.utcOffsetHours = utc_->value();
    s.site.latitudeDeg = lat_->value();
    s.site.longitudeDeg = lon_->value();
    s.site.northBearingDeg = north_->value();
    QString err;
    if (!viewer_->SetSun(s, &err)) info_->setText(err);
}

void SunPanel::refreshFromViewer() {
    const ModelViewer::SunSettings& s = viewer_->Sun();
    const bool was = updating_;
    updating_ = true;
    enabled_->setChecked(s.enabled);
    shadows_->setChecked(viewer_->ShadowsEnabled());
    time_->setDateTime(QDateTime(QDate(s.time.year, s.time.month, s.time.day), QTime(s.time.hour, s.time.minute, static_cast<int>(s.time.second))));
    utc_->setValue(s.time.utcOffsetHours);
    lat_->setValue(s.site.latitudeDeg);
    lon_->setValue(s.site.longitudeDeg);
    north_->setValue(s.site.northBearingDeg);
    updating_ = was;
    updateInfo();
}

void SunPanel::setNow() {
    const simul::env::SunTime t = simul::env::SunTimeNow();
    ModelViewer::SunSettings s = viewer_->Sun();
    s.time = t;
    QString err;
    if (!viewer_->SetSun(s, &err)) info_->setText(err);
}

void SunPanel::tick() {
    const double dtSec = static_cast<double>(clock_.restart()) / 1000.0;
    QString err;
    viewer_->SetSunTime(simul::env::AddMinutes(viewer_->Sun().time, speed_->value() * dtSec), &err);   // sunChanged 가 위젯을 갱신한다
}

void SunPanel::updateInfo() {
    const simul::env::SunState& st = viewer_->SunStateNow();
    const ModelViewer::SunSettings& s = viewer_->Sun();
    QString text = QString::fromUtf8("방위각 %1° (%2)   고도 %3°\n").arg(st.azimuthDeg, 0, 'f', 1).arg(BearingName(st.azimuthDeg)).arg(st.elevationDeg, 0, 'f', 1);
    text += st.aboveHorizon ? QString::fromUtf8("지금은 해가 떠 있음") : QString::fromUtf8("지금은 해가 진 상태 (밤)");
    double rise = 0, set = 0;
    bool polar = false;
    if (simul::env::SunriseSunset(s.time, s.site, &rise, &set, &polar)) text += QString::fromUtf8("\n일출 %1   일몰 %2 (현지 시각)").arg(HHMM(rise)).arg(HHMM(set));
    else text += polar ? QString::fromUtf8("\n이 날은 해가 지지 않음 (백야)") : QString::fromUtf8("\n이 날은 해가 뜨지 않음 (극야)");
    info_->setText(text);
}
