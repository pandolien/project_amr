#ifndef CORE_QT_QTSTRING_H
#define CORE_QT_QTSTRING_H

// amr::String(UTF-8) 와 QString 사이의 변환. string 을 거치지 않는다.
#include <QString>

#include "CBase/String.h"

namespace amrqt {

inline QString ToQ(const amr::String& s) { return QString::fromUtf8(s.c_str(), static_cast<int>(s.size())); }

inline amr::String FromQ(const QString& q) {
    const QByteArray b = q.toUtf8();
    return amr::String(b.constData(), static_cast<size_t>(b.size()));
}

// 파일 경로처럼 OS 의 로컬 인코딩이 필요한 곳에 쓴다
inline amr::String FromQLocal8Bit(const QString& q) {
    const QByteArray b = q.toLocal8Bit();
    return amr::String(b.constData(), static_cast<size_t>(b.size()));
}

}  // namespace amrqt

#endif  // CORE_QT_QTSTRING_H
