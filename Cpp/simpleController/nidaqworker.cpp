#if defined(__MINGW32__)
#define __int64 long long
#endif

#include <NIDAQmx.h>

#include "nidaqworker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace
{
void checkDaqmx(int32 status, const char *operation)
{
    if (DAQmxFailed(status)) {
        char message[2048] = {};
        DAQmxGetExtendedErrorInfo(message, sizeof(message));
        throw std::runtime_error(std::string(operation) + ": " + message);
    }
}

int32 CVICALLBACK everyNSamplesCallback(TaskHandle, int32, uInt32, void *callbackData)
{
    return static_cast<NiDaqWorker *>(callbackData)->processSamples();
}
}

NiDaqWorker::NiDaqWorker(QObject *parent) : QThread(parent)
{
}

void NiDaqWorker::setConfig(const ControlConfig &config)
{
    m_config = config;
    m_kp.store(config.kp, std::memory_order_relaxed);
    m_ki.store(config.ki, std::memory_order_relaxed);
}

void NiDaqWorker::setManualControl(bool enabled) noexcept
{
    const bool previous = m_manualControl.exchange(enabled, std::memory_order_relaxed);
    if (previous != enabled) {
        m_resetControllers.store(enabled, std::memory_order_relaxed);

        if (previous && !enabled) {
            m_resetSequence.store(true, std::memory_order_relaxed);
        }
    }
}

void NiDaqWorker::setManualReferences(double first, double second) noexcept
{
    m_manualReference1.store(first, std::memory_order_relaxed);
    m_manualReference2.store(second, std::memory_order_relaxed);
}

void NiDaqWorker::setCoefficients(double kp, double ki) noexcept
{
    m_kp.store(kp, std::memory_order_relaxed);
    m_ki.store(ki, std::memory_order_relaxed);
}

void NiDaqWorker::prepareForStart() noexcept
{
    m_stopRequested.store(false, std::memory_order_relaxed);
    m_finished.store(false, std::memory_order_relaxed);
    m_sequenceIndex = 0;
    m_iterationCount = 0;
}

void NiDaqWorker::requestStop() noexcept
{
    m_stopRequested.store(true, std::memory_order_relaxed);
    m_finishedCondition.notify_one();
}

void NiDaqWorker::run()
{
    try {
        const ControlConfig config = m_config;
        const int batchSize = config.ai_read_batch_size;

        m_controlDt = 1.0 / config.sample_rate;
        m_controlPeriod = batchSize * m_controlDt;
        m_inverseShunt = 1.0 / config.r_shunt;
        m_visualizationStride = std::max<std::size_t>(1, static_cast<std::size_t>(std::round(config.sample_rate / config.visualization_rate)));
        m_logBatchSamples = std::max<std::size_t>(1, static_cast<std::size_t>(std::round(config.sample_rate * config.log_emit_interval)));
        m_rateWindowCallbacks = 0;
        m_windowWorstInterval = 0.0;
        m_windowLateCallbacks = 0;
        m_lastCallbackStart = {};
        m_rateWindowStart = std::chrono::steady_clock::now();
        m_sequence = buildSequence(config);
        m_aiData.resize(static_cast<std::size_t>(batchSize) * 2);
        m_logBatch.clear();
        m_logBatch.reserve(static_cast<qsizetype>(m_logBatchSamples * 5));
        m_controller1 = PiController(config.kp, config.ki, config.min_voltage, config.max_voltage);
        m_controller2 = PiController(config.kp, config.ki, config.min_voltage, config.max_voltage);
        m_outputData[0] = 0.0;
        m_outputData[1] = 0.0;

        const std::string inputChannels = config.device_name + "/" + config.ai_channel;
        const std::string outputChannels = config.device_name + "/" + config.ao_channel;

        checkDaqmx(DAQmxCreateTask("SimpleController_AI", &m_aiTask), "Create AI task");
        checkDaqmx(DAQmxCreateAIVoltageChan(m_aiTask, inputChannels.c_str(), "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, nullptr), "Configure AI channels");
        checkDaqmx(DAQmxCfgSampClkTiming(m_aiTask, "", config.sample_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, static_cast<uInt64>(config.ai_buffer_size)), "Configure AI sample clock");
        checkDaqmx(DAQmxSetReadRelativeTo(m_aiTask, DAQmx_Val_CurrReadPos), "Configure AI read position");
        checkDaqmx(DAQmxSetReadOffset(m_aiTask, 0), "Configure AI read offset");

        checkDaqmx(DAQmxCreateTask("SimpleController_AO", &m_aoTask), "Create AO task");
        checkDaqmx(DAQmxCreateAOVoltageChan(m_aoTask, outputChannels.c_str(), "", config.min_voltage, config.max_voltage, DAQmx_Val_Volts, nullptr), "Configure AO channels");
        checkDaqmx(DAQmxTaskControl(m_aoTask, DAQmx_Val_Task_Commit), "Commit AO task");
        checkDaqmx(DAQmxStartTask(m_aoTask), "Start AO task");
        checkDaqmx(DAQmxWriteAnalogF64(m_aoTask, 1, FALSE, 0.05, DAQmx_Val_GroupByChannel, m_outputData, nullptr, nullptr), "Initialize AO outputs");
        checkDaqmx(DAQmxRegisterEveryNSamplesEvent(m_aiTask, DAQmx_Val_Acquired_Into_Buffer, static_cast<uInt32>(batchSize), 0, everyNSamplesCallback, this), "Register AI callback");

        m_nextVisualization = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / config.visualization_rate));
        emit timingUpdated(config.sample_rate / batchSize, 0.0, 0.0, 0);
        emit statusChanged(true);
        checkDaqmx(DAQmxStartTask(m_aiTask), "Start AI task");

        std::unique_lock<std::mutex> lock(m_waitMutex);

        m_finishedCondition.wait(lock, [this]() {
            return m_stopRequested.load(std::memory_order_relaxed) || m_finished.load(std::memory_order_relaxed);
        });
        lock.unlock();

        m_outputData[0] = 0.0;
        m_outputData[1] = 0.0;

        if (m_aoTask != nullptr) {
            DAQmxWriteAnalogF64(m_aoTask, 1, FALSE, 0.05, DAQmx_Val_GroupByChannel, m_outputData, nullptr, nullptr);
        }

        if (m_aiTask != nullptr) {
            DAQmxStopTask(m_aiTask);
            DAQmxClearTask(m_aiTask);
            m_aiTask = nullptr;
        }
        if (m_aoTask != nullptr) {
            DAQmxStopTask(m_aoTask);
            DAQmxClearTask(m_aoTask);
            m_aoTask = nullptr;
        }
        if (!m_logBatch.isEmpty()) {
            emit batchReady(std::move(m_logBatch));
        }
        if (!m_callbackError.empty()) {
            emit errorOccurred(QString::fromStdString(m_callbackError));
        }
    } catch (const std::exception &exception) {
        if (m_aiTask != nullptr) {
            DAQmxStopTask(m_aiTask);
            DAQmxClearTask(m_aiTask);
            m_aiTask = nullptr;
        }
        if (m_aoTask != nullptr) {
            m_outputData[0] = 0.0;
            m_outputData[1] = 0.0;
            DAQmxWriteAnalogF64(m_aoTask, 1, FALSE, 0.05, DAQmx_Val_GroupByChannel, m_outputData, nullptr, nullptr);
            DAQmxStopTask(m_aoTask);
            DAQmxClearTask(m_aoTask);
            m_aoTask = nullptr;
        }
        emit errorOccurred(QString::fromUtf8(exception.what()));
    }
    emit statusChanged(false);
}

int NiDaqWorker::processSamples() noexcept
{
    if (m_stopRequested.load(std::memory_order_relaxed) || m_finished.load(std::memory_order_relaxed)) {
        return 0;
    }

    if (m_resetSequence.exchange(false, std::memory_order_relaxed)) {
        m_sequenceIndex = 0;
    }

    if (m_resetControllers.exchange(false, std::memory_order_relaxed)) {
        m_controller1.reset();
        m_controller2.reset();
    }

    const auto callbackStart = std::chrono::steady_clock::now();

    if (m_lastCallbackStart.time_since_epoch().count() != 0) {
        const double interval = std::chrono::duration<double>(callbackStart - m_lastCallbackStart).count();
        m_windowWorstInterval = std::max(m_windowWorstInterval, interval);
        if (interval > 2.0 * m_controlPeriod) {
            ++m_windowLateCallbacks;
        }
    }
    m_lastCallbackStart = callbackStart;

    const int batchSize = m_config.ai_read_batch_size;
    int32 samplesRead = 0;
    const int32 readStatus = DAQmxReadAnalogF64(m_aiTask, batchSize, 0.01, DAQmx_Val_GroupByChannel, m_aiData.data(), static_cast<uInt32>(m_aiData.size()), &samplesRead, nullptr);

    if (DAQmxFailed(readStatus)) {
        setCallbackError("Read AI samples", readStatus);
        return 0;
    }
    if (samplesRead != batchSize) {
        setCallbackError("Read AI samples", DAQmxErrorSamplesNotYetAvailable);
        return 0;
    }

    const bool manual = m_manualControl.load(std::memory_order_relaxed);
    if (!manual && m_sequenceIndex + static_cast<std::size_t>(batchSize) > m_sequence.size()) {
        m_finished.store(true, std::memory_order_relaxed);
        m_finishedCondition.notify_one();
        return 0;
    }

    double sum1 = 0.0;
    double sum2 = 0.0;

    for (int sample = 0; sample < batchSize; ++sample) {
        sum1 += m_aiData[static_cast<std::size_t>(sample)];
        sum2 += m_aiData[static_cast<std::size_t>(batchSize + sample)];
    }
    const double measurement1 = sum1 / batchSize * m_inverseShunt;
    const double measurement2 = sum2 / batchSize * m_inverseShunt;
    const ReferencePoint reference = manual
                                         ? ReferencePoint{m_manualReference1.load(std::memory_order_relaxed), m_manualReference2.load(std::memory_order_relaxed)}
                                         : m_sequence[m_sequenceIndex + static_cast<std::size_t>(batchSize - 1)];

    const double kp = m_kp.load(std::memory_order_relaxed);
    const double ki = m_ki.load(std::memory_order_relaxed);

    m_controller1.setCoefficients(kp, ki);
    m_controller2.setCoefficients(kp, ki);

    m_outputData[0] = m_controller1.process(reference.first, measurement1, m_controlPeriod);
    m_outputData[1] = m_controller2.process(reference.second, measurement2, m_controlPeriod);

    const int32 writeStatus = DAQmxWriteAnalogF64(m_aoTask, 1, FALSE, 0.01, DAQmx_Val_GroupByChannel, m_outputData, nullptr, nullptr);
    if (DAQmxFailed(writeStatus)) {
        setCallbackError("Write AO sample", writeStatus);
        return 0;
    }

    ++m_rateWindowCallbacks;
    const double rateWindowSeconds = std::chrono::duration<double>(callbackStart - m_rateWindowStart).count();
    if (rateWindowSeconds >= 1.0) {
        const double measuredHz = m_rateWindowCallbacks / rateWindowSeconds;
        emit timingUpdated(m_config.sample_rate / batchSize, measuredHz, m_windowWorstInterval * 1000.0, m_windowLateCallbacks);
        m_rateWindowCallbacks = 0;
        m_windowWorstInterval = 0.0;
        m_windowLateCallbacks = 0;
        m_rateWindowStart = callbackStart;
    }

    const std::size_t startIndex = m_sequenceIndex;
    const double startTime = static_cast<double>(m_iterationCount) * m_controlDt;
    for (int sample = 0; sample < batchSize; ++sample) {
        const ReferencePoint loggedReference = manual ? reference : m_sequence[startIndex + static_cast<std::size_t>(sample)];
        const std::size_t dataIndex = static_cast<std::size_t>(sample);
        m_logBatch.append(startTime + static_cast<double>(sample) * m_controlDt);
        m_logBatch.append(loggedReference.first);
        m_logBatch.append(m_aiData[dataIndex] * m_inverseShunt);
        m_logBatch.append(loggedReference.second);
        m_logBatch.append(m_aiData[static_cast<std::size_t>(batchSize) + dataIndex] * m_inverseShunt);
    }
    if (!manual) {
        m_sequenceIndex += static_cast<std::size_t>(batchSize);
    }
    m_iterationCount += static_cast<std::size_t>(batchSize);
    if (m_logBatch.size() >= static_cast<qsizetype>(m_logBatchSamples * 5)) {
        emit batchReady(std::move(m_logBatch));
        m_logBatch = QVector<double>();
        m_logBatch.reserve(static_cast<qsizetype>(m_logBatchSamples * 5));
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= m_nextVisualization) {
        emit sampleUpdated(reference.first, measurement1, reference.second, measurement2, static_cast<double>(m_iterationCount) * m_controlDt);
        m_nextVisualization = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / m_config.visualization_rate));
    }
    return 0;
}

void NiDaqWorker::setCallbackError(const char *operation, int errorCode) noexcept
{
    char message[2048] = {};
    DAQmxGetExtendedErrorInfo(message, sizeof(message));
    {
        std::lock_guard<std::mutex> lock(m_waitMutex);
        m_callbackError = std::string(operation) + " (" + std::to_string(errorCode) + "): " + message;
    }
    m_finished.store(true, std::memory_order_relaxed);
    m_finishedCondition.notify_one();
}