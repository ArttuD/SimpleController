#include "mainwindow.h"

#include "csvsaver.h"
#include "plotwidget.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSlider>
#include <QStringList>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QDoubleValidator>
#include <algorithm>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), m_niWorker(), m_saver(new CsvSaver(this))
{
    setWindowTitle(QStringLiteral("Current Controller | NI-DAQmx"));
    resize(1200, 760);

    auto *tabs = new QTabWidget(this);
    auto *graphPage = new QWidget(tabs);
    auto *graphLayout = new QVBoxLayout(graphPage);
    auto *controlRow = new QHBoxLayout;
    auto *actions = new QVBoxLayout;
    auto *startButton = new QPushButton(QStringLiteral("Start"), graphPage);
    auto *stopButton = new QPushButton(QStringLiteral("Stop"), graphPage);
    auto *clearButton = new QPushButton(QStringLiteral("Clear"), graphPage);
    auto *closeButton = new QPushButton(QStringLiteral("Close"), graphPage);
    actions->addWidget(startButton);
    actions->addWidget(stopButton);
    actions->addWidget(clearButton);
    actions->addWidget(closeButton);
    actions->addStretch();
    controlRow->addLayout(actions);

    auto *manualGroup = new QGroupBox(QStringLiteral("Manual reference"), graphPage);
    auto *manualLayout = new QVBoxLayout(manualGroup);
    m_manualControl = new QCheckBox(QStringLiteral("Enable manual control"), manualGroup);
    manualLayout->addWidget(m_manualControl);
    const auto addCurrentSlider = [manualGroup, manualLayout](const QString &name, QSlider *&slider, QLabel *&valueLabel) {
        auto *row = new QHBoxLayout;
        row->addWidget(new QLabel(name, manualGroup));
        slider = new QSlider(Qt::Horizontal, manualGroup);
        slider->setRange(-500, 500);
        slider->setValue(0);
        row->addWidget(slider, 1);
        valueLabel = new QLabel(QStringLiteral("0.00 A"), manualGroup);
        valueLabel->setMinimumWidth(68);
        row->addWidget(valueLabel);
        manualLayout->addLayout(row);
    };
    addCurrentSlider(QStringLiteral("Current 1"), m_current1Slider, m_current1Value);
    addCurrentSlider(QStringLiteral("Current 2"), m_current2Slider, m_current2Value);
    controlRow->addWidget(manualGroup, 2);

    auto *piGroup = new QGroupBox(QStringLiteral("Controller"), graphPage);
    auto *piForm = new QFormLayout(piGroup);
    m_kp = new QDoubleSpinBox(piGroup);
    m_ki = new QDoubleSpinBox(piGroup);
    m_kff = new QDoubleSpinBox(piGroup);
    m_rShunt = new QDoubleSpinBox(piGroup);
    for (auto *spin : {m_kp, m_ki}) {
        spin->setRange(-100000.0, 100000.0);
        spin->setDecimals(4);
        spin->setSingleStep(0.1);
    }
    m_kff->setRange(-1000.0, 1000.0);
    m_kff->setDecimals(4);
    m_kff->setSingleStep(0.01);
    m_kff->setSuffix(QStringLiteral(" V/A"));
    m_rShunt->setRange(0.000001, 1000.0);
    m_rShunt->setDecimals(6);
    m_rShunt->setSingleStep(0.001);
    m_rShunt->setSuffix(QStringLiteral(" ohm"));
    // Otherwise every keystroke reaches the hardware, e.g. typing 20 sends 2 first.
    for (auto *spin : {m_kp, m_ki, m_kff, m_rShunt}) {
        spin->setKeyboardTracking(false);
    }
    m_kp->setValue(m_config.kp);
    m_ki->setValue(m_config.ki);
    m_kff->setValue(m_config.kff);
    m_rShunt->setValue(m_config.r_shunt);
    piForm->addRow(QStringLiteral("Kp"), m_kp);
    piForm->addRow(QStringLiteral("Ki"), m_ki);
    piForm->addRow(QStringLiteral("Kff"), m_kff);
    piForm->addRow(QStringLiteral("R shunt"), m_rShunt);
    controlRow->addWidget(piGroup);

    auto *readoutGroup = new QGroupBox(QStringLiteral("Readout"), graphPage);
    auto *readoutForm = new QFormLayout(readoutGroup);
    m_measurement1 = new QLabel(QStringLiteral("0.00 A"), readoutGroup);
    m_measurement2 = new QLabel(QStringLiteral("0.00 A"), readoutGroup);
    m_runStatus = new QLabel(QStringLiteral("Stopped"), readoutGroup);
    readoutForm->addRow(QStringLiteral("Measured 1"), m_measurement1);
    readoutForm->addRow(QStringLiteral("Measured 2"), m_measurement2);
    readoutForm->addRow(QStringLiteral("Status"), m_runStatus);
    controlRow->addWidget(readoutGroup);
    graphLayout->addLayout(controlRow);

    m_timingStatus = new QLabel(QStringLiteral("Control rate: idle"), graphPage);
    graphLayout->addWidget(m_timingStatus);
    m_softwareStatus = new QPlainTextEdit(graphPage);
    m_softwareStatus->setReadOnly(true);
    m_softwareStatus->setMaximumHeight(76);
    graphLayout->addWidget(m_softwareStatus);
    m_plot = new PlotWidget(graphPage);
    graphLayout->addWidget(m_plot, 1);
    tabs->addTab(graphPage, QStringLiteral("Current graph"));

    auto *settingsPage = new QWidget(tabs);
    auto *settingsLayout = new QVBoxLayout(settingsPage);
    auto *settingsContent = new QWidget(settingsPage);
    auto *settingsForm = new QFormLayout(settingsContent);
    m_sequenceMode = new QComboBox(settingsContent);
    m_sequenceMode->addItems({QStringLiteral("Oscillatory test"), QStringLiteral("Pulse"), QStringLiteral("Sawtooth")});
    settingsForm->addRow(QStringLiteral("Current sequence"), m_sequenceMode);

    const auto addTextEditor = [this, settingsContent, settingsForm](const QString &label, std::string ControlConfig::*member) {
        auto *line = new QLineEdit(QString::fromStdString(m_config.*member), settingsContent);
        settingsForm->addRow(label, line);
        m_configEditors.append({line, nullptr, nullptr, member, label});
    };
    const auto addRealEditor = [this, settingsContent, settingsForm](const QString &label, double ControlConfig::*member) {
        auto *line = new QLineEdit(QString::number(m_config.*member, 'g', 12), settingsContent);
        line->setValidator(new QDoubleValidator(-1.0e12, 1.0e12, 8, line));
        settingsForm->addRow(label, line);
        m_configEditors.append({line, member, nullptr, nullptr, label});
    };
    const auto addIntegerEditor = [this, settingsContent, settingsForm](const QString &label, int ControlConfig::*member) {
        auto *line = new QLineEdit(QString::number(m_config.*member), settingsContent);
        line->setValidator(new QIntValidator(0, 100000000, line));
        settingsForm->addRow(label, line);
        m_configEditors.append({line, nullptr, member, nullptr, label});
    };

    addTextEditor(QStringLiteral("Device name"), &ControlConfig::device_name);
    addTextEditor(QStringLiteral("AI channel"), &ControlConfig::ai_channel);
    addTextEditor(QStringLiteral("AO channel"), &ControlConfig::ao_channel);
    addRealEditor(QStringLiteral("Sample rate [Hz]"), &ControlConfig::sample_rate);
    addIntegerEditor(QStringLiteral("AI buffer size"), &ControlConfig::ai_buffer_size);
    addIntegerEditor(QStringLiteral("AI/AO batch size"), &ControlConfig::ai_read_batch_size);
    addRealEditor(QStringLiteral("Shunt resistance [ohm]"), &ControlConfig::r_shunt);
    addRealEditor(QStringLiteral("Minimum voltage [V]"), &ControlConfig::min_voltage);
    addRealEditor(QStringLiteral("Maximum voltage [V]"), &ControlConfig::max_voltage);
    addRealEditor(QStringLiteral("Measurement filter [Hz] (0 = off)"), &ControlConfig::measurement_filter_hz);
    addRealEditor(QStringLiteral("Peak amplitude coil 1 [A]"), &ControlConfig::peak_amplitude_POS);
    addRealEditor(QStringLiteral("Peak amplitude coil 2 [A]"), &ControlConfig::peak_amplitude_NEG);
    addRealEditor(QStringLiteral("Peak width [s]"), &ControlConfig::peak_width);
    addRealEditor(QStringLiteral("Peak interval [s]"), &ControlConfig::peak_interval);
    addIntegerEditor(QStringLiteral("Peak number"), &ControlConfig::peak_number);
    addRealEditor(QStringLiteral("Pre-magnetization pause [s]"), &ControlConfig::pre_magnetization_pause);
    addRealEditor(QStringLiteral("Magnetization period [s]"), &ControlConfig::magnetization_period);
    addRealEditor(QStringLiteral("Magnetization amplitude [A]"), &ControlConfig::magnetization_amplitude);
    addRealEditor(QStringLiteral("Sine frequency [Hz]"), &ControlConfig::sine_frequency);
    addRealEditor(QStringLiteral("Sine offset [A]"), &ControlConfig::sine_offset);
    addRealEditor(QStringLiteral("Sine amplitude [A]"), &ControlConfig::sine_amplitude);
    addRealEditor(QStringLiteral("Sine periods"), &ControlConfig::sine_periods);
    addRealEditor(QStringLiteral("Post-sequence pause [s]"), &ControlConfig::post_sequence_pause);
    addRealEditor(QStringLiteral("Visualization rate [Hz]"), &ControlConfig::visualization_rate);
    addRealEditor(QStringLiteral("Diagnostics interval [s]"), &ControlConfig::diagnostics_interval);
    addRealEditor(QStringLiteral("Log batch interval [s]"), &ControlConfig::log_emit_interval);
    addRealEditor(QStringLiteral("Pulse amplitude [A]"), &ControlConfig::pulse_amplitude);
    addRealEditor(QStringLiteral("Pulse period [s]"), &ControlConfig::pulse_period);
    addRealEditor(QStringLiteral("Pulse width [s]"), &ControlConfig::pulse_width);
    addIntegerEditor(QStringLiteral("Pulse cycles"), &ControlConfig::pulse_cycles);
    addRealEditor(QStringLiteral("Sawtooth minimum [A]"), &ControlConfig::sawtooth_min);
    addRealEditor(QStringLiteral("Sawtooth maximum [A]"), &ControlConfig::sawtooth_max);
    addRealEditor(QStringLiteral("Sawtooth period [s]"), &ControlConfig::sawtooth_period);
    addIntegerEditor(QStringLiteral("Sawtooth cycles"), &ControlConfig::sawtooth_cycles);
    m_filename = new QLineEdit(QStringLiteral("data/measurements.csv"), settingsContent);
    settingsForm->addRow(QStringLiteral("CSV filename"), m_filename);

    auto *scroll = new QScrollArea(settingsPage);
    scroll->setWidgetResizable(true);
    scroll->setWidget(settingsContent);
    settingsLayout->addWidget(scroll, 1);
    auto *applyButton = new QPushButton(QStringLiteral("Apply settings"), settingsPage);
    settingsLayout->addWidget(applyButton);
    tabs->addTab(settingsPage, QStringLiteral("Settings"));
    setCentralWidget(tabs);

    connect(startButton, &QPushButton::clicked, this, &MainWindow::startControl);
    connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopControl);
    connect(clearButton, &QPushButton::clicked, this, &MainWindow::clearMeasurements);
    connect(closeButton, &QPushButton::clicked, this, &QWidget::close);
    connect(applyButton, &QPushButton::clicked, this, [this]() { applySettings(true); });
    connect(m_manualControl, &QCheckBox::toggled, this, [this](bool enabled) {
        m_niWorker.setManualControl(enabled);
        if (!enabled) {
            m_current1Slider->setValue(0);
            m_current2Slider->setValue(0);
        }
        appendStatus(enabled ? QStringLiteral("Manual override enabled") : QStringLiteral("Automatic sequence enabled"));
    });
    const auto updateManualReferences = [this](int) {
        const double first = m_current1Slider->value() / 100.0;
        const double second = m_current2Slider->value() / 100.0;
        m_current1Value->setText(QString::number(first, 'f', 2) + QStringLiteral(" A"));
        m_current2Value->setText(QString::number(second, 'f', 2) + QStringLiteral(" A"));
        m_niWorker.setManualReferences(first, second);
    };
    connect(m_current1Slider, &QSlider::valueChanged, this, updateManualReferences);
    connect(m_current2Slider, &QSlider::valueChanged, this, updateManualReferences);
    connect(m_kp, &QDoubleSpinBox::valueChanged, this, [this](double) { m_niWorker.setCoefficients(m_kp->value(), m_ki->value()); });
    connect(m_ki, &QDoubleSpinBox::valueChanged, this, [this](double) { m_niWorker.setCoefficients(m_kp->value(), m_ki->value()); });
    connect(m_kff, &QDoubleSpinBox::valueChanged, this, [this](double value) {
        m_niWorker.setKff(value);
        appendStatus(QStringLiteral("Feedforward set to %1 V/A").arg(value, 0, 'g', 6));
    });
    connect(m_rShunt, &QDoubleSpinBox::valueChanged, this, [this](double value) {
        m_niWorker.setRShunt(value);
        for (const ConfigEditor &editor : m_configEditors) {
            if (editor.real == &ControlConfig::r_shunt) {
                editor.line->setText(QString::number(value, 'g', 12));
            }
        }
        appendStatus(QStringLiteral("Shunt resistance set to %1 ohm").arg(value, 0, 'g', 6));
    });

    connect(&m_niWorker, &NiDaqWorker::sampleUpdated, m_plot, &PlotWidget::appendSample);
    connect(&m_niWorker, &NiDaqWorker::sampleUpdated, this, [this](double, double measurement1, double, double measurement2, double) {
        m_measurement1->setText(QString::number(measurement1, 'f', 2) + QStringLiteral(" A"));
        m_measurement2->setText(QString::number(measurement2, 'f', 2) + QStringLiteral(" A"));
    });
    connect(&m_niWorker, &NiDaqWorker::batchReady, m_saver, [this](QVector<double> samples) { m_saver->enqueue(std::move(samples)); }, Qt::DirectConnection);
    connect(&m_niWorker, &NiDaqWorker::statusChanged, this, [this](bool running) {
        m_runStatus->setText(running ? QStringLiteral("Running") : QStringLiteral("Stopped"));
        appendStatus(running ? QStringLiteral("NI-DAQmx control started") : QStringLiteral("NI-DAQmx control stopped"));
    });
    connect(&m_niWorker, &NiDaqWorker::errorOccurred, this, [this](const QString &message) {
        m_manualControl->setChecked(false);
        m_runStatus->setText(QStringLiteral("ERROR"));
        appendStatus(QStringLiteral("NI-DAQmx error: ") + message);
    });
    connect(&m_niWorker, &NiDaqWorker::timingUpdated, this, [this](double targetHz, double measuredHz, double worstIntervalMs, quint64 lateCallbacks) {
        const QString measured = measuredHz > 0.0 ? QString::number(measuredHz, 'f', 1) + QStringLiteral(" Hz") : QStringLiteral("measuring...");
        m_timingStatus->setText(QStringLiteral("Control rate: target %1 Hz, measured %2, worst callback interval %3 ms, late (>2x period) %4")
                                     .arg(targetHz, 0, 'f', 1)
                                     .arg(measured)
                                     .arg(worstIntervalMs, 0, 'f', 3)
                                     .arg(lateCallbacks));
    });
    connect(m_saver, &CsvSaver::statusChanged, this, &MainWindow::appendStatus);
    connect(m_saver, &CsvSaver::errorOccurred, this, [this](const QString &message) { appendStatus(message); });
    m_niWorker.setConfig(m_config);
    m_saver->openFile(m_filename->text());
    appendStatus(QStringLiteral("Application started"));
}

MainWindow::~MainWindow()
{
    stopControl();
    m_saver->shutdown();
}

bool MainWindow::applySettings(bool restartIfRunning)
{
    ControlConfig candidate = m_config;
    candidate.sequence_type = m_sequenceMode->currentText().toStdString();
    candidate.kp = m_kp->value();
    candidate.ki = m_ki->value();
    candidate.kff = m_kff->value();
    for (const ConfigEditor &editor : m_configEditors) {
        if (editor.real) {
            bool valid = false;
            const double value = editor.line->text().toDouble(&valid);
            if (!valid || !std::isfinite(value)) {
                appendStatus(QStringLiteral("Settings error: enter a finite numeric value"));
                return false;
            }
            candidate.*(editor.real) = value;
        } else if (editor.integer) {
            bool valid = false;
            const int value = editor.line->text().toInt(&valid);
            if (!valid) {
                appendStatus(QStringLiteral("Settings error: enter a whole-number value"));
                return false;
            }
            candidate.*(editor.integer) = value;
        } else if (editor.text) {
            candidate.*(editor.text) = editor.line->text().trimmed().toStdString();
        }
    }
    if (candidate.sample_rate <= 0.0 || candidate.sample_rate > 1000000.0) {
        appendStatus(QStringLiteral("Settings error: sample rate must be in (0, 1,000,000] Hz"));
        return false;
    }
    if (candidate.ai_read_batch_size <= 0 || candidate.ai_buffer_size < candidate.ai_read_batch_size) {
        appendStatus(QStringLiteral("Settings error: buffer size must be at least one positive batch"));
        return false;
    }
    if (candidate.visualization_rate <= 0.0 || candidate.log_emit_interval <= 0.0 || candidate.sample_rate * candidate.log_emit_interval > 1000000.0 || candidate.r_shunt <= 0.0 || candidate.max_voltage <= candidate.min_voltage) {
        appendStatus(QStringLiteral("Settings error: rates, shunt resistance, and voltage range must be positive"));
        return false;
    }
    if (candidate.measurement_filter_hz < 0.0) {
        appendStatus(QStringLiteral("Settings error: measurement filter cannot be negative"));
        return false;
    }
    if (m_filename->text().trimmed().isEmpty()) {
        appendStatus(QStringLiteral("Settings error: CSV filename cannot be empty"));
        return false;
    }

    QStringList changes;
    if (candidate.sequence_type != m_config.sequence_type) {
        changes.append(QStringLiteral("Current sequence: %1 -> %2").arg(QString::fromStdString(m_config.sequence_type), QString::fromStdString(candidate.sequence_type)));
    }
    for (const ConfigEditor &editor : m_configEditors) {
        if (editor.real && m_config.*(editor.real) != candidate.*(editor.real)) {
            changes.append(QStringLiteral("%1: %2 -> %3").arg(editor.label, QString::number(m_config.*(editor.real), 'g', 12), QString::number(candidate.*(editor.real), 'g', 12)));
        } else if (editor.integer && m_config.*(editor.integer) != candidate.*(editor.integer)) {
            changes.append(QStringLiteral("%1: %2 -> %3").arg(editor.label, QString::number(m_config.*(editor.integer)), QString::number(candidate.*(editor.integer))));
        } else if (editor.text && m_config.*(editor.text) != candidate.*(editor.text)) {
            changes.append(QStringLiteral("%1: %2 -> %3").arg(editor.label, QString::fromStdString(m_config.*(editor.text)), QString::fromStdString(candidate.*(editor.text))));
        }
    }

    const bool wasRunning = m_niWorker.isRunning();
    if (wasRunning) {
        stopControl();
    }
    m_config = candidate;
    m_kp->setValue(m_config.kp);
    m_ki->setValue(m_config.ki);
    {
        const QSignalBlocker blocker(m_rShunt);
        m_rShunt->setValue(m_config.r_shunt);
    }
    m_niWorker.setConfig(m_config);
    m_saver->openFile(m_filename->text().trimmed());
    if (!changes.isEmpty()) {
        appendStatus(QStringLiteral("Changed: ") + changes.join(QStringLiteral(", ")));
    }
    appendStatus(QStringLiteral("Settings applied; CSV: ") + m_filename->text().trimmed());
    if (wasRunning && restartIfRunning) {
        startControl();
    }
    return true;
}

void MainWindow::startControl()
{
    if (m_niWorker.isRunning()) {
        return;
    }
    if (!applySettings(false)) {
        return;
    }
    clearMeasurements();
    m_niWorker.setManualControl(m_manualControl->isChecked());
    m_niWorker.setManualReferences(m_current1Slider->value() / 100.0, m_current2Slider->value() / 100.0);
    m_niWorker.setCoefficients(m_kp->value(), m_ki->value());
    m_niWorker.prepareForStart();
    m_niWorker.start();
    appendStatus(QStringLiteral("Starting NI-DAQmx controller"));
}

void MainWindow::stopControl()
{
    if (m_niWorker.isRunning()) {
        m_niWorker.requestStop();
        m_niWorker.wait();
    }
    m_niWorker.setManualControl(false);
    m_manualControl->setChecked(false);
    m_current1Slider->setValue(0);
    m_current2Slider->setValue(0);
    clearMeasurements();
}

void MainWindow::clearMeasurements()
{
    m_plot->clear();
    m_measurement1->setText(QStringLiteral("0.00 A"));
    m_measurement2->setText(QStringLiteral("0.00 A"));
    appendStatus(QStringLiteral("Cleared measurements"));
}

void MainWindow::appendStatus(const QString &message)
{
    m_softwareStatus->appendPlainText(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")) + QStringLiteral(" | ") + message);
    m_softwareStatus->verticalScrollBar()->setValue(m_softwareStatus->verticalScrollBar()->maximum());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    stopControl();
    m_saver->shutdown();
    event->accept();
}
