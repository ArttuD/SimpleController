#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <string>

#include "controller.h"
#include "nidaqworker.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QSlider;
class PlotWidget;
class CsvSaver;

QT_BEGIN_NAMESPACE
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    struct ConfigEditor
    {
        QLineEdit *line = nullptr;
        double ControlConfig::*real = nullptr;
        int ControlConfig::*integer = nullptr;
        std::string ControlConfig::*text = nullptr;
        QString label;
    };

    bool applySettings(bool restartIfRunning);
    void startControl();
    void stopControl();
    void clearMeasurements();
    void appendStatus(const QString &message);
    void closeEvent(QCloseEvent *event) override;

    ControlConfig m_config;
    NiDaqWorker m_niWorker;
    CsvSaver *m_saver;
    QVector<ConfigEditor> m_configEditors;
    QComboBox *m_sequenceMode;
    QLineEdit *m_filename;
    QCheckBox *m_manualControl;
    QSlider *m_current1Slider;
    QSlider *m_current2Slider;

    QLabel *m_current1Value;
    QLabel *m_current2Value;

    QLabel *m_measurement1;
    QLabel *m_measurement2;

    QLabel *m_runStatus;
    QLabel *m_timingStatus;
    QDoubleSpinBox *m_kp;
    QDoubleSpinBox *m_ki;
    QDoubleSpinBox *m_kff;
    QDoubleSpinBox *m_rShunt_1;
    QDoubleSpinBox *m_rShunt_2;
    QPlainTextEdit *m_softwareStatus;
    PlotWidget *m_plot;
};
#endif // MAINWINDOW_H
