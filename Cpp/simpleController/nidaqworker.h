#ifndef NIDAQWORKER_H
#define NIDAQWORKER_H

#include "controller.h"

#include <QThread>
#include <QVector>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

class NiDaqWorker : public QThread
{
    Q_OBJECT

public:
    explicit NiDaqWorker(QObject *parent = nullptr);

    void setConfig(const ControlConfig &config);
    void setManualControl(bool enabled) noexcept;
    void setManualReferences(double first, double second) noexcept;
    void setCoefficients(double kp, double ki) noexcept;
    void setKff(double kff) noexcept;
    void setRShunt(double rShunt) noexcept;
    void prepareForStart() noexcept;
    void requestStop() noexcept;
    int processSamples() noexcept;

signals:
    void sampleUpdated(double reference1, double measurement1, double reference2, double measurement2, double timeSeconds);
    void batchReady(QVector<double> flattenedSamples);
    void statusChanged(bool running);
    void errorOccurred(QString message);
    void timingUpdated(double targetHz, double measuredHz, double worstIntervalMs, quint64 lateCallbacks);

protected:
    void run() override;

private:
    void setCallbackError(const char *operation, int errorCode) noexcept;

    ControlConfig m_config;
    void *m_aiTask = nullptr;
    void *m_aoTask = nullptr;
    std::vector<ReferencePoint> m_sequence;
    std::vector<double> m_aiData;
    QVector<double> m_logBatch;
    PiController m_controller1{20.0, 10.0, -10.0, 10.0};
    PiController m_controller2{20.0, 10.0, -10.0, 10.0};
    double m_outputData[2] = {0.0, 0.0};
    std::size_t m_sequenceIndex = 0;
    std::size_t m_iterationCount = 0;
    std::size_t m_visualizationStride = 1;
    std::size_t m_logBatchSamples = 1;
    std::size_t m_rateWindowCallbacks = 0;
    double m_controlDt = 0.001;
    double m_controlPeriod = 0.001;
    double m_filterAlpha = 1.0;
    double m_filtered1 = 0.0;
    double m_filtered2 = 0.0;
    double m_windowWorstInterval = 0.0;
    quint64 m_windowLateCallbacks = 0;
    std::atomic_bool m_stopRequested{false};
    std::atomic_bool m_finished{false};
    std::atomic_bool m_manualControl{false};
    std::atomic_bool m_resetSequence{false};
    std::atomic_bool m_resetControllers{false};
    std::atomic<double> m_manualReference1{0.0};
    std::atomic<double> m_manualReference2{0.0};
    std::atomic<double> m_kp{20.0};
    std::atomic<double> m_ki{10.0};
    std::atomic<double> m_kff{0.0};
    std::atomic<double> m_inverseShunt{1.0};
    std::mutex m_waitMutex;
    std::condition_variable m_finishedCondition;
    std::string m_callbackError;
    std::chrono::steady_clock::time_point m_lastCallbackStart;
    std::chrono::steady_clock::time_point m_rateWindowStart;
    std::chrono::steady_clock::time_point m_nextVisualization;
};

#endif