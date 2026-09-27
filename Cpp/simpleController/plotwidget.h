#ifndef PLOTWIDGET_H
#define PLOTWIDGET_H

#include <QPointF>
#include <QVector>
#include <QWidget>

class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PlotWidget(QWidget *parent = nullptr);
    void clear();

public slots:
    void appendSample(double reference1, double measurement1, double reference2, double measurement2, double timeSeconds);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QVector<QPointF> m_reference1;
    QVector<QPointF> m_measurement1;
    QVector<QPointF> m_reference2;
    QVector<QPointF> m_measurement2;
};

#endif