#include "controller.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr std::size_t maxSequenceSamples = 50000000;

std::size_t sampleCount(double seconds, double rate)
{
    if (!std::isfinite(seconds) || seconds < 0.0 || !std::isfinite(rate) || rate <= 0.0) {
        throw std::invalid_argument("Sequence durations and sample rate must be finite and non-negative");
    }

    const double count = std::round(seconds * rate);
    if (count > static_cast<double>(maxSequenceSamples)) {
        throw std::length_error("Sequence exceeds the 50-million-sample limit");
    }
    return static_cast<std::size_t>(count);
}

void appendConstant(std::vector<ReferencePoint> &sequence, std::size_t count, double first, double second)
{
    sequence.insert(sequence.end(), count, ReferencePoint{first, second});
}

void appendPeakCycles(std::vector<ReferencePoint> &sequence, const ControlConfig &config, double rate)
{
    const std::size_t intervalSamples = sampleCount(config.peak_interval, rate);
    const std::size_t widthSamples = sampleCount(config.peak_width, rate);
    const std::size_t cycleSamples = sampleCount(config.peak_interval + config.peak_width, rate);
    const std::size_t pulseSamples = std::min(widthSamples, cycleSamples - std::min(cycleSamples, intervalSamples));

    for (int cycle = 0; cycle < config.peak_number; ++cycle) {
        appendConstant(sequence, std::min(cycleSamples, intervalSamples), 0.0, 0.0);
        appendConstant(sequence, pulseSamples, config.peak_amplitude_POS, config.peak_amplitude_NEG);
        appendConstant(sequence, cycleSamples - std::min(cycleSamples, intervalSamples) - pulseSamples, 0.0, 0.0);
    }
}
}

PiController::PiController(double kp, double ki, double outputMin, double outputMax)
    : m_kp(kp), m_ki(ki), m_outputMin(outputMin), m_outputMax(outputMax)
{
}

void PiController::setCoefficients(double kp, double ki) noexcept
{
    m_kp = kp;
    m_ki = ki;
}

void PiController::reset() noexcept
{
    m_integral = 0.0;
}

double PiController::process(double reference, double measurement, double dt, double feedforward) noexcept
{
    const double error = reference - measurement;
    const double candidateIntegral = m_integral + error * dt;
    const double rawOutput = feedforward + m_kp * error + m_ki * candidateIntegral;

    if (rawOutput > m_outputMax) {
        if (error < 0.0) {
            m_integral = candidateIntegral;
        }
        return m_outputMax;
    }
    if (rawOutput < m_outputMin) {
        if (error > 0.0) {
            m_integral = candidateIntegral;
        }
        return m_outputMin;
    }

    m_integral = candidateIntegral;
    return rawOutput;
}

std::vector<ReferencePoint> buildSequence(const ControlConfig &config)
{
    if (!std::isfinite(config.sample_rate) || config.sample_rate <= 0.0) {
        throw std::invalid_argument("Sample rate must be positive");
    }

    const double rate = config.sample_rate;
    std::vector<ReferencePoint> sequence;

    if (config.sequence_type == "Pulse") {
        const std::size_t periodSamples = std::max<std::size_t>(1, sampleCount(config.pulse_period, rate));
        const std::size_t widthSamples = std::min(periodSamples, std::max<std::size_t>(1, sampleCount(config.pulse_width, rate)));
        const std::size_t pauseSamples = sampleCount(config.post_sequence_pause, rate);
        const std::size_t cycles = static_cast<std::size_t>(std::max(0, config.pulse_cycles));

        if (cycles * periodSamples + pauseSamples > maxSequenceSamples) {
            throw std::length_error("Sequence exceeds the 50-million-sample limit");
        }
        sequence.reserve(cycles * periodSamples + pauseSamples);
        for (std::size_t cycle = 0; cycle < cycles; ++cycle) {
            appendConstant(sequence, widthSamples, config.pulse_amplitude, config.pulse_amplitude);
            appendConstant(sequence, periodSamples - widthSamples, 0.0, 0.0);
        }
        appendConstant(sequence, pauseSamples, 0.0, 0.0);
        return sequence;
    }

    if (config.sequence_type == "Sawtooth") {
        const std::size_t periodSamples = std::max<std::size_t>(1, sampleCount(config.sawtooth_period, rate));
        const std::size_t pauseSamples = sampleCount(config.post_sequence_pause, rate);
        const std::size_t cycles = static_cast<std::size_t>(std::max(0, config.sawtooth_cycles));
        if (cycles * periodSamples + pauseSamples > maxSequenceSamples) {
            throw std::length_error("Sequence exceeds the 50-million-sample limit");
        }
        sequence.reserve(cycles * periodSamples + pauseSamples);
        for (std::size_t cycle = 0; cycle < cycles; ++cycle) {
            for (std::size_t sample = 0; sample < periodSamples; ++sample) {
                const double fraction = static_cast<double>(sample) / static_cast<double>(periodSamples);
                const double value = config.sawtooth_min + (config.sawtooth_max - config.sawtooth_min) * fraction;
                sequence.push_back({value, value});
            }
        }
        appendConstant(sequence, pauseSamples, 0.0, 0.0);
        return sequence;
    }

    if (config.sine_frequency <= 0.0 || !std::isfinite(config.sine_frequency)) {
        throw std::invalid_argument("Sine frequency must be positive");
    }

    const std::size_t peaks = static_cast<std::size_t>(std::max(0, config.peak_number))
                              * sampleCount(config.peak_interval + config.peak_width, rate);
    const std::size_t prePauseSamples = sampleCount(config.pre_magnetization_pause, rate);
    const std::size_t magnetizationSamples = sampleCount(config.magnetization_period, rate);
    const std::size_t sineSamples = sampleCount(config.sine_periods / config.sine_frequency, rate);
    const std::size_t postPauseSamples = sampleCount(config.post_sequence_pause, rate);
    const std::size_t totalSamples = peaks * 2 + prePauseSamples + magnetizationSamples + sineSamples + postPauseSamples;
    if (totalSamples > maxSequenceSamples) {
        throw std::length_error("Sequence exceeds the 50-million-sample limit");
    }

    sequence.reserve(totalSamples);
    appendPeakCycles(sequence, config, rate);
    appendConstant(sequence, prePauseSamples, 0.0, 0.0);
    appendConstant(sequence, magnetizationSamples, config.magnetization_amplitude, config.magnetization_amplitude);

    for (std::size_t sample = 0; sample < sineSamples; ++sample) {
        const double time = static_cast<double>(sample) / rate;
        const double wave = config.sine_amplitude * std::sin(2.0 * pi * config.sine_frequency * time);
        sequence.push_back({config.sine_offset + wave, config.sine_offset - wave});
    }

    appendPeakCycles(sequence, config, rate);
    appendConstant(sequence, postPauseSamples, 0.0, 0.0);
    return sequence;
}