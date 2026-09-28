#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <string>

struct ControlConfig
{
    std::string device_name = "Dev3";
    std::string ai_channel = "ai0:1";
    std::string ao_channel = "ao0:1";
    double sample_rate = 1000.0;
    std::string sequence_type = "Oscillatory test";
    int ai_buffer_size = 100000;
    int ai_read_batch_size = 1;
    double r_shunt = 0.12;
    double min_voltage = -10.0;
    double max_voltage = 10.0;
    double kp = 20.0;
    double ki = 10.0;
    double kff = 0.0;
    double measurement_filter_hz = 0.0;
    double peak_amplitude_POS = 2.0;
    double peak_amplitude_NEG = -0.5;
    double peak_width = 1.0;
    double peak_interval = 2.0;
    int peak_number = 3;
    double pre_magnetization_pause = 0.5;
    double magnetization_period = 5.0;
    double magnetization_amplitude = 0.75;
    double sine_frequency = 0.05;
    double sine_offset = 0.75;
    double sine_amplitude = 1.25;
    double sine_periods = 1.5;
    double post_sequence_pause = 1.0;
    double pulse_amplitude = 2.0;
    double pulse_period = 2.0;
    double pulse_width = 0.5;
    int pulse_cycles = 10;
    double sawtooth_min = 0.0;
    double sawtooth_max = 2.0;
    double sawtooth_period = 10.0;
    int sawtooth_cycles = 2;
    double visualization_rate = 10.0;
    double diagnostics_interval = 2.0;
    double log_emit_interval = 0.2;
};

struct ReferencePoint
{
    double first = 0.0;
    double second = 0.0;
};

class PiController
{
public:
    PiController(double kp, double ki, double output_min, double output_max);

    void setCoefficients(double kp, double ki) noexcept;
    void reset() noexcept;
    double process(double reference, double measurement, double dt, double feedforward = 0.0) noexcept;

private:
    double m_kp;
    double m_ki;
    double m_outputMin;
    double m_outputMax;
    double m_integral = 0.0;
};

std::vector<ReferencePoint> buildSequence(const ControlConfig &config);

#endif