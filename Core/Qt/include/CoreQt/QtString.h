#ifndef CORE_QT_QTSTRING_H
#define CORE_QT_QTSTRING_H

// simul::String(UTF-8) 와 QString 사이의 변환. string 을 거치지 않는다.
#include <QString>

#include "CBase/String.h"

namespace simulqt {

inline QString ToQ(const simul::String& s) { return QString::fromUtf8(s.c_str(), static_cast<int>(s.size())); }

inline simul::String FromQ(const QString& q) {
    const QByteArray b = q.toUtf8();
    return simul::String(b.constData(), static_cast<size_t>(b.size()));
}

// 파일 경로처럼 OS 의 로컬 인코딩이 필요한 곳에 쓴다
inline simul::String FromQLocal8Bit(const QString& q) {
    const QByteArray b = q.toLocal8Bit();
    return simul::String(b.constData(), static_cast<size_t>(b.size()));
}

}  // namespace simulqt

#endif  // CORE_QT_QTSTRING_H
