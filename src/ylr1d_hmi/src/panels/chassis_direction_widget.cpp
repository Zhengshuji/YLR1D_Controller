#include "ylr1d_hmi/panels/translate_panel.hpp"

#include <QFont>
#include <QLineF>
#include <QPainter>
#include <QPolygonF>
#include <QRectF>

#include <algorithm>
#include <cmath>

namespace ylr1d_hmi {

ChassisDirectionWidget::ChassisDirectionWidget(QWidget * parent)
  : QWidget(parent) {
  setMinimumHeight(120);
}

void ChassisDirectionWidget::setParams(int mode, double direction, double speed) {
  mode_ = mode;
  direction_ = direction;
  speed_ = speed;
  update();
}

void ChassisDirectionWidget::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const double cx = width() / 2.0;
  const double cy = height() / 2.0;
  const double R = qMin(width(), height()) / 2.0 - 10.0;
  if (R < 20) return;

  // 背景圆盘 + 参考十字
  p.setPen(QPen(QColor(0xd2, 0xd2, 0xd2), 1));
  p.setBrush(QColor(0xf6, 0xf6, 0xf6));
  p.drawEllipse(QPointF(cx, cy), R, R);
  p.setPen(QPen(QColor(0xcc, 0xcc, 0xcc), 1, Qt::DashLine));
  p.drawLine(QPointF(cx - R, cy), QPointF(cx + R, cy));
  p.drawLine(QPointF(cx, cy - R), QPointF(cx, cy + R));
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0x66, 0x66, 0x66));
  p.drawEllipse(QPointF(cx, cy), 3, 3);

  const double sp = speed_;
  if (mode_ == 0) {  // Translate
    if (std::abs(sp) < 1e-6) {
      p.setPen(QPen(QColor(0x99, 0x99, 0x99), 1, Qt::DotLine));
      p.drawEllipse(QPointF(cx, cy), R * 0.25, R * 0.25);
      return;
    }
    const double maxSp = 2.5;  // m/s，与面板 speed 上限一致
    const double len = 12.0 + (std::min(std::abs(sp), maxSp) / maxSp) * (R - 24.0);
    const double a = direction_;
    const double sgn = sp >= 0 ? 1.0 : -1.0;
    const QPointF from(cx, cy);
    // direction=0 指向屏幕上方（车头朝上）；π/2 指向左（视觉逆时针）
    const QPointF to(cx - std::sin(a) * len * sgn, cy - std::cos(a) * len * sgn);
    drawLineArrow(p, from, to, QColor(0x2d, 0x6c, 0xdf));
  } else if (mode_ == 1) {  // Rotate
    if (std::abs(sp) < 1e-6) {
      p.setPen(QPen(QColor(0x99, 0x99, 0x99), 1, Qt::DotLine));
      p.drawEllipse(QPointF(cx, cy), R * 0.25, R * 0.25);
      return;
    }
    const double maxSp = 5.0;  // rad/s，与面板 speed 上限一致
    const double f = std::min(std::abs(sp), maxSp) / maxSp;
    const double span = 60.0 + f * 60.0;   // 弧扫过角度（度）
    const double radius = R * 0.55;
    const int start16 = -90 * 16;          // 顶部起
    // Qt drawArc 正 sweep = 逆时针；正角速度 → 逆时针
    const int sweep16 = sp >= 0 ? static_cast<int>(span * 16.0)
                                : static_cast<int>(-span * 16.0);
    drawArcArrow(p, QPointF(cx, cy), radius, start16, sweep16, QColor(0xe0, 0x7b, 0x00));
  } else {  // Stop
    const double s = 14.0;
    p.setPen(QPen(QColor(0xc6, 0x28, 0x28), 2));
    p.setBrush(QColor(0xc6, 0x28, 0x28));
    p.drawRect(QRectF(cx - s, cy - s, 2 * s, 2 * s));
    p.setPen(QColor(0xff, 0xff, 0xff));
    QFont f = p.font();
    f.setBold(true);
    f.setPointSize(9);
    p.setFont(f);
    p.drawText(QRectF(cx - s, cy - s, 2 * s, 2 * s), Qt::AlignCenter, QStringLiteral("STOP"));
  }
}

void ChassisDirectionWidget::drawLineArrow(QPainter & p, const QPointF & from,
                                           const QPointF & to, const QColor & color) {
  QLineF line(from, to);
  const double ang = std::atan2(line.dy(), line.dx());  // 屏幕角（Y 向下）
  const double as = 12.0;
  const QPointF a1(to.x() - as * std::cos(ang - 0.45), to.y() - as * std::sin(ang - 0.45));
  const QPointF a2(to.x() - as * std::cos(ang + 0.45), to.y() - as * std::sin(ang + 0.45));
  p.setPen(QPen(color, 2.5, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(line);
  p.setPen(Qt::NoPen);
  p.setBrush(color);
  QPolygonF tri;
  tri << to << a1 << a2;
  p.drawPolygon(tri);
}

void ChassisDirectionWidget::drawArcArrow(QPainter & p, const QPointF & center, double radius,
                                          int start16, int sweep16, const QColor & color) {
  const QRectF arcRect(center.x() - radius, center.y() - radius, 2 * radius, 2 * radius);
  p.setPen(QPen(color, 3, Qt::SolidLine, Qt::RoundCap));
  p.drawArc(arcRect, start16, sweep16);

  const double endDeg = (start16 + sweep16) / 16.0;
  const double endRad = endDeg * M_PI / 180.0;
  const QPointF tip(center.x() + radius * std::cos(endRad),
                    center.y() - radius * std::sin(endRad));
  const double tang = (endDeg + 90.0) * M_PI / 180.0;  // 弧继续方向的屏幕角
  const double as = 11.0;
  const QPointF a1(tip.x() - as * std::cos(tang + 0.4), tip.y() + as * std::sin(tang + 0.4));
  const QPointF a2(tip.x() - as * std::cos(tang - 0.4), tip.y() + as * std::sin(tang - 0.4));
  p.setPen(Qt::NoPen);
  p.setBrush(color);
  QPolygonF tri;
  tri << tip << a1 << a2;
  p.drawPolygon(tri);
}

}  // namespace ylr1d_hmi
