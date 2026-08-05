#ifndef YLR1D_HMI__COMMON__TOPIC_STATUS_HPP_
#define YLR1D_HMI__COMMON__TOPIC_STATUS_HPP_

#include <QString>

#include <chrono>

namespace ylr1d_hmi {

/// Per-topic receive bookkeeping, shown in every sensor view so the panel
/// reports whether a topic is alive, how long ago it last updated and the
/// observed receive rate — independent of whether the data renders.
struct TopicStatus {
  std::chrono::steady_clock::time_point first{};
  std::chrono::steady_clock::time_point last{};
  size_t count{0};

  void touch() {
    const auto now = std::chrono::steady_clock::now();
    if (count++ == 0) first = now;
    last = now;
  }

  /// Seconds since the last message, or -1 if none yet.
  double ageSeconds() const {
    if (count == 0) return -1.0;
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last).count();
  }

  /// Observed receive rate (Hz), or -1 if unknown.
  double rateHz() const {
    if (count <= 1) return -1.0;
    const double span = std::chrono::duration<double>(last - first).count();
    if (span <= 0.0) return -1.0;
    return (static_cast<double>(count) - 1.0) / span;
  }

  /// "got N msgs · updated Xs ago · ~Y Hz" — or a "no message" note when
  /// nothing arrived yet. English only (WSL Qt lacks CJK fonts → mojibake).
  QString text() const {
    if (count == 0) return QStringLiteral("no message received");
    const double age = ageSeconds();
    QString s = QStringLiteral("got %1 msgs · updated %2s ago")
                    .arg(count).arg(age, 0, 'f', 1);
    const double rate = rateHz();
    if (rate >= 0.0)
      s += QStringLiteral(" · ~%1 Hz").arg(rate, 0, 'f', 1);
    return s;
  }
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__COMMON__TOPIC_STATUS_HPP_
