"""Settings editor for VainoDriver control and sequence configuration."""

from dataclasses import replace

from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from src.niClient import ControlConfig


class SettingsWidget(QWidget):

    settings_changed = pyqtSignal(object, str)
    status_message = pyqtSignal(str)

    FIELD_SPECS = (
        ("device_name", "Device name", str),
        ("ai_channel", "AI channel", str),
        ("ao_channel", "AO channel", str),
        ("sample_rate", "Sample rate [Hz]", float),
        ("ai_buffer_size", "AI buffer size", int),
        ("ai_read_batch_size", "AI/AO batch size", int),
        ("r_shunt", "Shunt resistance [ohm]", float),
        ("min_voltage", "Minimum voltage [V]", float),
        ("max_voltage", "Maximum voltage [V]", float),
        ("measurement_filter_hz", "Measurement filter [Hz] (0 = off)", float),
        ("peak_amplitude_POS", "Peak amplitude coil 1 [A]", float),
        ("peak_amplitude_NEG", "Peak amplitude coil 2 [A]", float),
        ("peak_width", "Peak width [s]", float),
        ("peak_interval", "Peak interval [s]", float),
        ("peak_number", "Peak number", int),
        (
            "pre_magnetization_pause",
            "Pre-magnetization pause [s]",
            float,
        ),
        ("magnetization_period", "Magnetization period [s]", float),
        (
            "magnetization_amplitude",
            "Magnetization amplitude [A]",
            float,
        ),
        ("sine_frequency", "Sine frequency [Hz]", float),
        ("sine_offset", "Sine offset [A]", float),
        ("sine_amplitude", "Sine amplitude [A]", float),
        ("sine_periods", "Sine periods", float),
        ("post_sequence_pause", "Post-sequence pause [s]", float),
        ("visualization_rate", "Visualization rate [Hz]", float),
        ("diagnostics_interval", "Diagnostics interval [s]", float),
        ("pulse_amplitude", "Pulse amplitude [A]", float),
        ("pulse_period", "Pulse period [s]", float),
        ("pulse_width", "Pulse width [s]", float),
        ("pulse_cycles", "Pulse cycles", int),
        ("sawtooth_min", "Sawtooth minimum [A]", float),
        ("sawtooth_max", "Sawtooth maximum [A]", float),
        ("sawtooth_period", "Sawtooth period [s]", float),
        ("sawtooth_cycles", "Sawtooth cycles", int),
    )

    def __init__(self, config=None, filename="data/measurements.csv", parent=None):

        super().__init__(parent)

        self.config = config or ControlConfig()
        self.fields = {}
        self.types = {
            name: value_type
            for name, _label, value_type in self.FIELD_SPECS
        }

        outer_layout = QVBoxLayout(self)
        form = QFormLayout()

        self.sequence_mode = QComboBox()
        self.sequence_mode.addItem(
            "Oscillatory test",
            "Oscillatory test",
        )
        self.sequence_mode.addItem("Pulse", "Pulse")
        self.sequence_mode.addItem("Sawtooth", "Sawtooth")
        form.addRow("Current sequence", self.sequence_mode)

        for name, label, _value_type in self.FIELD_SPECS:
            editor = QLineEdit()
            self.fields[name] = editor
            form.addRow(label, editor)

        self.filename = QLineEdit()
        form.addRow("CSV filename", self.filename)

        self.apply_button = QPushButton("Apply settings")
        self.apply_button.clicked.connect(self.apply_settings)

        self.message = QLabel()
        self.message.setWordWrap(True)

        form_widget = QWidget()
        form_widget.setLayout(form)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(form_widget)

        outer_layout.addWidget(scroll)
        outer_layout.addWidget(self.apply_button)
        outer_layout.addWidget(self.message)

        self.set_config(self.config, filename)

    def set_config(self, config, filename):

        self.config = config
        sequence_type = config.sequence_type
        if sequence_type == "Calibration":
            sequence_type = "Oscillatory test"

        sequence_index = self.sequence_mode.findData(sequence_type)
        if sequence_index >= 0:
            self.sequence_mode.setCurrentIndex(sequence_index)

        for name, editor in self.fields.items():
            editor.setText(str(getattr(config, name)))

        self.filename.setText(str(filename))

    @pyqtSlot()
    def apply_settings(self):

        try:
            values = {}
            for name, editor in self.fields.items():
                value_type = self.types[name]
                text = editor.text().strip()
                if value_type is str:
                    values[name] = text
                else:
                    values[name] = value_type(text)

            if values["sample_rate"] <= 0.0:
                raise ValueError("Sample rate must be positive")
            if values["ai_buffer_size"] < values["ai_read_batch_size"]:
                raise ValueError("AI buffer must be at least one batch")
            if values["ai_read_batch_size"] <= 0:
                raise ValueError("Batch size must be positive")
            if values["visualization_rate"] <= 0.0:
                raise ValueError("Visualization rate must be positive")
            if values["r_shunt"] <= 0.0:
                raise ValueError("Shunt resistance must be positive")
            if values["measurement_filter_hz"] < 0.0:
                raise ValueError("Measurement filter cannot be negative")
            if not self.filename.text().strip():
                raise ValueError("CSV filename cannot be empty")

            config = replace(
                self.config,
                sequence_type=self.sequence_mode.currentData(),
                **values,
            )
            self.config = config
            filename = self.filename.text().strip()
            self.settings_changed.emit(config, filename)
            self.status_message.emit("Settings applied")
            self.message.setText("Settings applied")

        except (TypeError, ValueError) as exc:
            message = f"Settings error: {exc}"
            self.status_message.emit(message)
            self.message.setText(message)
