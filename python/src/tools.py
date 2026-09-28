import logging

from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QSlider,
)


# ============================================================
# Logging
# ============================================================

def cfg_logging():
    """
    Configure and return the application logger.

    Safe to call multiple times.
    """

    logger = logging.getLogger("VainoDriver")

    if not logger.handlers:

        logger.setLevel(logging.DEBUG)

        formatter = logging.Formatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        console_handler.setFormatter(formatter)

        logger.addHandler(console_handler)

        # Prevent duplicate output through the root logger.
        logger.propagate = False

    return logger


# ============================================================
# ViewBox
# ============================================================

class ViewBox(QWidget):

    value_changed = pyqtSignal(float)

    def __init__(
        self,
        name,
        text,
        units,
        start_value=0.0,
        editable=False,
        parent=None,
        decimals=2,
    ):

        super().__init__(parent)

        self.name = name
        self.text = text
        self.units = units
        self.editable = editable
        self.decimals = int(decimals)

        self.value = float(start_value)

        layout = QHBoxLayout(self)

        self.label = QLabel(text)

        layout.addWidget(
            self.label
        )

        self.value_field = QLineEdit(
            f"{self.value:.{self.decimals}f}",
        )

        self.value_field.setAlignment(
            Qt.AlignmentFlag.AlignHCenter
        )

        self.value_field.setReadOnly(
            not editable
        )

        layout.addWidget(
            self.value_field
        )

        self.units_label = QLabel(
            units
        )

        layout.addWidget(
            self.units_label
        )

        if editable:

            self.value_field.editingFinished.connect(
                self._editing_finished
            )

    # --------------------------------------------------------
    # Editing
    # --------------------------------------------------------

    def _editing_finished(self):

        try:

            value = float(
                self.value_field.text()
            )

        except (
            ValueError,
            TypeError,
        ):

            value = self.value

        self.set_value(
            value,
            emit_signal=True,
        )

    # --------------------------------------------------------
    # Set
    # --------------------------------------------------------

    def set_value(
        self,
        value,
        emit_signal=False,
    ):

        try:

            value = float(value)

        except (
            ValueError,
            TypeError,
        ):

            value = 0.0

        self.value = value

        self.value_field.setText(
            f"{value:.{self.decimals}f}"
        )

        if emit_signal:

            self.value_changed.emit(
                value
            )

    # --------------------------------------------------------
    # Get
    # --------------------------------------------------------

    def get_value(self):

        return float(
            self.value
        )

    # --------------------------------------------------------
    # Reset
    # --------------------------------------------------------

    def reset(self):

        self.set_value(
            0.0
        )


# ============================================================
# Current Slider
# ============================================================

class CurrentSlider(QWidget):

    current_changed = pyqtSignal(float)

    def __init__(
        self,
        text,
        min_current,
        max_current,
        start_value=0.0,
        parent=None,
    ):

        super().__init__(parent)

        self.min_current = float(
            min_current
        )

        self.max_current = float(
            max_current
        )

        # 0.01 A resolution.
        self.scale = 100

        layout = QHBoxLayout(self)

        self.label = QLabel(
            text
        )

        layout.addWidget(
            self.label
        )

        self.slider = QSlider(
            Qt.Orientation.Horizontal
        )

        self.slider.setMinimum(
            int(
                round(
                    self.min_current
                    * self.scale
                )
            )
        )

        self.slider.setMaximum(
            int(
                round(
                    self.max_current
                    * self.scale
                )
            )
        )

        self.slider.setValue(
            int(
                round(
                    start_value
                    * self.scale
                )
            )
        )

        layout.addWidget(
            self.slider
        )

        self.value_label = QLabel(
            "0.00 A"
        )

        self.value_label.setMinimumWidth(
            70
        )

        layout.addWidget(
            self.value_label
        )

        self.slider.valueChanged.connect(
            self._slider_changed
        )

        self._slider_changed(
            self.slider.value()
        )

    # --------------------------------------------------------
    # Slider changed
    # --------------------------------------------------------

    @pyqtSlot(int)
    def _slider_changed(
        self,
        value,
    ):

        current = (
            float(value)
            / self.scale
        )

        self.value_label.setText(
            f"{current:.2f} A"
        )

        self.current_changed.emit(
            current
        )

    # --------------------------------------------------------
    # Set
    # --------------------------------------------------------

    def set_value(
        self,
        current,
    ):

        try:

            current = float(
                current
            )

        except (
            ValueError,
            TypeError,
        ):

            current = 0.0

        current = max(
            self.min_current,
            min(
                current,
                self.max_current,
            ),
        )

        self.slider.setValue(
            int(
                round(
                    current
                    * self.scale
                )
            )
        )

    # --------------------------------------------------------
    # Get
    # --------------------------------------------------------

    def get_value(self):

        return (
            self.slider.value()
            / self.scale
        )

    # --------------------------------------------------------
    # Reset
    # --------------------------------------------------------

    def reset(self):

        self.set_value(
            0.0
        )
