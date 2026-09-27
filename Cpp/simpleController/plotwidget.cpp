#include "plotwidget.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>

PlotWidget::PlotWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumHeight(260);
    setAutoFillBackground(false);
}

void PlotWidget::clear()
{
    m_reference1.clear();
    m_measurement1.clear();
    m_reference2.clear();
    m_measurement2.clear();
    update();
}

void PlotWidget::appendSample(double reference1, double measurement1, double reference2, double measurement2, double timeSeconds)
{
    constexpr qsizetype maxPoints = 100;
    if (m_reference1.size() == maxPoints) {
        m_reference1.removeFirst();
        m_measurement1.removeFirst();
        m_reference2.removeFirst();
        m_measurement2.removeFirst();
    }
    m_reference1.append({timeSeconds, reference1});
    m_measurement1.append({timeSeconds, measurement1});
    m_reference2.append({timeSeconds, reference2});
    m_measurement2.append({timeSeconds, measurement2});
    update();
}

void PlotWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor("#fbfcfd"));

    const QRectF chart = QRectF(rect()).adjusted(68.0, 20.0, -18.0, -62.0);
    painter.setPen(QPen(QColor("#d8dee5"), 1.0));
    for (int line = 0; line <= 4; ++line) {
        const double y = chart.top() + chart.height() * line / 4.0;
        painter.drawLine(QPointF(chart.left(), y), QPointF(chart.right(), y));
        const double value = 4.0 - line * 2.0;
        painter.setPen(QColor("#586575"));
        painter.drawText(QRectF(26.0, y - 10.0, 34.0, 20.0), Qt::AlignRight | Qt::AlignVCenter, QString::number(value));
        painter.setPen(QPen(QColor("#d8dee5"), 1.0));
    }
    painter.setPen(QColor("#586575"));
    painter.save();
    painter.translate(14.0, chart.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-chart.height() / 2.0, -12.0, chart.height(), 24.0), Qt::AlignCenter, "Current [A]");
    painter.restore();

    const double minTime = m_reference1.isEmpty() ? 0.0 : m_reference1.first().x();
    const double maxTime = m_reference1.size() < 2 ? minTime + 1.0 : m_reference1.last().x();
    const double timeSpan = std::max(1e-9, maxTime - minTime);
    painter.setPen(QPen(QColor("#d8dee5"), 1.0));
    for (int tick = 0; tick <= 4; ++tick) {
        const double fraction = tick / 4.0;
        const double x = chart.left() + chart.width() * fraction;
        painter.drawLine(QPointF(x, chart.top()), QPointF(x, chart.bottom()));
        painter.setPen(QColor("#586575"));
        painter.drawText(QRectF(x - 34.0, chart.bottom() + 3.0, 68.0, 18.0), Qt::AlignHCenter | Qt::AlignVCenter, QString::number(minTime + timeSpan * fraction, 'g', 3));
        painter.setPen(QPen(QColor("#d8dee5"), 1.0));
    }
    painter.setPen(QColor("#586575"));
    painter.drawText(QRectF(chart.left(), chart.bottom() + 21.0, chart.width(), 18.0), Qt::AlignCenter, "Time [s]");
    const auto mapPoint = [&chart, minTime, timeSpan](const QPointF &point) {
        const double x = chart.left() + (point.x() - minTime) / timeSpan * chart.width();
        const double y = chart.bottom() - (point.y() + 4.0) / 8.0 * chart.height();
        return QPointF(x, y);
    };
    const auto drawCurve = [&painter, &mapPoint](const QVector<QPointF> &points, const QColor &color, Qt::PenStyle style) {
        if (points.size() < 2) {
            return;
        }
        QPainterPath path(mapPoint(points.first()));
        for (qsizetype index = 1; index < points.size(); ++index) {
            path.lineTo(mapPoint(points[index]));
        }
        painter.setPen(QPen(color, 2.0, style));
        painter.drawPath(path);
    };

    painter.save();
    painter.setClipRect(chart);
    drawCurve(m_measurement1, QColor("#1687a7"), Qt::SolidLine);
    drawCurve(m_reference1, QColor("#d94f45"), Qt::DashLine);
    drawCurve(m_measurement2, QColor("#438c61"), Qt::SolidLine);
    drawCurve(m_reference2, QColor("#d39a22"), Qt::DashLine);
    painter.restore();

    const QVector<QPair<QString, QColor>> legend{{"Measured 1", QColor("#1687a7")}, {"Reference 1", QColor("#d94f45")}, {"Measured 2", QColor("#438c61")}, {"Reference 2", QColor("#d39a22")}};
    int legendX = qRound(chart.left());
    for (const auto &item : legend) {
        painter.setPen(QPen(item.second, 2.0));
        painter.drawLine(legendX, height() - 10, legendX + 18, height() - 10);
        painter.setPen(QColor("#3d4855"));
        painter.drawText(legendX + 24, height() - 6, item.first);
        legendX += 112;
    }
}