#ifndef YLR1D_HMI__COMMON__ACTION_SENDER_HPP_
#define YLR1D_HMI__COMMON__ACTION_SENDER_HPP_

#include <rclcpp_action/rclcpp_action.hpp>

#include <QTime>
#include <QLabel>
#include <QPlainTextEdit>
#include <QString>

#include <chrono>

namespace ylr1d_hmi {
namespace detail {

/// Replicates TranslatePanel::appendLog: "HH:mm:ss  message".
inline void appendPlain(QPlainTextEdit * view, const QString & msg) {
  if (!view) return;
  view->appendPlainText(QTime::currentTime().toString(QStringLiteral("HH:mm:ss"))
                        + QStringLiteral("  ") + msg);
}

/// Replicates TranslatePanel::setStatus: colored <font>.
inline void setStatusHtml(QLabel * lbl, const QString & text, bool ok) {
  if (!lbl) return;
  lbl->setText(QStringLiteral("<font color='%1'>%2</font>")
                   .arg(ok ? QStringLiteral("#2e7d32") : QStringLiteral("#c62828"), text));
}

}  // namespace detail

/// Wait for an action server (1 s). On failure writes the exact same status
/// + log lines the original per-sender code did; returns false.
template <typename ActionT>
bool waitForActionServer(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const QString & action_name,   // log prefix, no leading slash, e.g. "chassis_move"
    QLabel * status_lbl, QPlainTextEdit * log_view) {
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    detail::setStatusHtml(status_lbl,
                          QStringLiteral("ERROR: /%1 server not found").arg(action_name), false);
    detail::appendPlain(log_view, action_name + QStringLiteral(": ERROR - server not found"));
    return false;
  }
  return true;
}

/// Build SendGoalOptions with unified goal_response + result callbacks whose
/// log lines match the original per-sender output byte-for-byte. The caller
/// still assigns opts.feedback_callback before sending.
template <typename ActionT>
typename rclcpp_action::Client<ActionT>::SendGoalOptions
makeGoalOptions(const QString & action_name,   // log prefix, no leading slash
                QLabel * status_lbl, QPlainTextEdit * log_view) {
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = typename Client::GoalHandle;

  typename Client::SendGoalOptions opts;
  opts.goal_response_callback =
      [action_name, status_lbl, log_view](const typename GoalHandle::SharedPtr & gh) {
        if (gh) {
          const auto & id = gh->get_goal_id();
          detail::appendPlain(log_view,
                              QStringLiteral("%1: goal accepted id=%2%3")
                                  .arg(action_name)
                                  .arg(id[0], 2, 16, QLatin1Char('0'))
                                  .arg(id[1], 2, 16, QLatin1Char('0')));
        } else {
          detail::appendPlain(log_view, action_name + QStringLiteral(": goal rejected"));
        }
      };
  opts.result_callback =
      [action_name, status_lbl, log_view](const typename GoalHandle::WrappedResult & res) {
        if (res.result) {
          const bool ok = res.result->success;
          const auto text = QStringLiteral("result success=%1 msg=%2")
                                .arg(ok ? QStringLiteral("true") : QStringLiteral("false"))
                                .arg(QString::fromStdString(res.result->message));
          detail::setStatusHtml(status_lbl, text, ok);
          detail::appendPlain(log_view, action_name + QStringLiteral(": ") + text);
        } else {
          const auto text =
              QStringLiteral("goal aborted (code=%1)").arg(static_cast<int>(res.code));
          detail::setStatusHtml(status_lbl, text, false);
          detail::appendPlain(log_view, action_name + QStringLiteral(": ") + text);
        }
      };
  return opts;
}

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__COMMON__ACTION_SENDER_HPP_
