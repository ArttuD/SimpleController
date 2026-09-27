import sys
import logging
from datetime import datetime
from pathlib import Path

import pyqtgraph as pg

from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QHBoxLayout,
    QMainWindow,
    QPlainTextEdit,
    QTabWidget,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from src.niClient import NiClient
from src.saverClient import SaverClient
from src.settingsWidget import SettingsWidget

from src.tools import (
    CurrentSlider,
    ViewBox,
    cfg_logging,
)


class MainWindow(QMainWindow):

    def __init__(
        self,
    ):

        super().__init__()

        self.setWindowTitle("Current Controller")

        self.resize(1200,700,)

        self.logger = cfg_logging()
        self.logger.info("Application started")

        self.timestamp = []
        self.meas_data_1 = []
        self.set_data_1 = []
        self.meas_data_2 = []
        self.set_data_2 = []

        self.max_points = 100
        self.current_controls = {}


        self.niClient = NiClient()
        self.niClient.data_updated.connect(self.update_plot)
        self.niClient.error.connect(self.ni_error)
        self.niClient.status_changed.connect(self.ni_status_changed)

        self.saverClient = SaverClient()
        self.saverClient.status_changed.connect(self.saver_status_changed)
        self.saverClient.error.connect(self.saver_error)
        self.niClient.samples_updated.connect(self.saverClient.save_batch)



        self._setup_ui()
        self.clear_measurements()

        self.saverClient.start(Path("data") / "measurements.csv")


    def _setup_ui(self,):

        tabs = QTabWidget()
        graph_page = QWidget()

        main_layout = QVBoxLayout(graph_page)
        control_layout = QHBoxLayout()

        button_layout = QVBoxLayout()

        self.btn_start = QPushButton("Start")
        self.btn_stop = QPushButton("Stop")
        self.btn_clear = QPushButton("Clear")
        self.btn_close = QPushButton("Close")

        self.btn_start.clicked.connect(self.start_control)
        self.btn_stop.clicked.connect(self.stop_control)
        self.btn_clear.clicked.connect(self.clear_measurements)
        self.btn_close.clicked.connect(self.close)

        button_layout.addWidget(self.btn_start)
        button_layout.addWidget(self.btn_stop)
        button_layout.addWidget(self.btn_clear)
        button_layout.addWidget(self.btn_close)
        control_layout.addLayout(button_layout)


        manual_layout = QVBoxLayout()
        self.enable_manual = QCheckBox("Enable Manual Control")
        self.enable_manual.toggled.connect(self.on_manual_changed)
        manual_layout.addWidget(self.enable_manual)

        self.current_slider_1 = CurrentSlider("Current 1",-5.0,5.0,0.0,)
        self.current_slider_2 = CurrentSlider("Current 2",-5.0,5.0,0.0,)

        self.current_controls["current_slider_1"] = self.current_slider_1
        self.current_controls["current_slider_2"] = self.current_slider_2

        self.current_slider_1.current_changed.connect(self.on_current_slider_changed)
        self.current_slider_2.current_changed.connect(self.on_current_slider_changed)

        manual_layout.addWidget(self.current_slider_1)
        manual_layout.addWidget(self.current_slider_2)
        control_layout.addLayout(manual_layout)


        pi_layout = QVBoxLayout()

        self.kp = ViewBox("kp","Kp","",self.niClient.config.kp,editable=True,)
        self.ki = ViewBox("ki","Ki","",self.niClient.config.ki,editable=True,)

        self.kp.value_changed.connect(self.update_gains)
        self.ki.value_changed.connect(self.update_gains)

        pi_layout.addWidget(self.kp)
        pi_layout.addWidget(self.ki)

        control_layout.addLayout(pi_layout)

        readout_layout = QVBoxLayout()

        self.current_1_view = ViewBox("current_1","Measured current 1","A",0.0,editable=False,)
        self.current_2_view = ViewBox("current_2","Measured current 2","A",0.0,editable=False,)

        readout_layout.addWidget(self.current_1_view)
        readout_layout.addWidget(self.current_2_view)

        control_layout.addLayout(readout_layout)

        self.status_label = ViewBox("status","Status","",0.0,editable=False,)
        self.status_label.value_field.hide()
        self.status_label.units_label.hide()
        self.status_label.value_field.setText("Stopped")
        control_layout.addWidget(self.status_label)

        main_layout.addLayout(control_layout)

        self.software_status = QPlainTextEdit()
        self.software_status.setReadOnly(True)
        self.software_status.setMaximumHeight(80)
        self.software_status.setPlaceholderText("Software status")

        main_layout.addWidget(self.software_status)

        self.plot = pg.PlotWidget()

        self.plot.setTitle("Current Response")

        self.plot.setLabel("left","Current",units="A",)
        self.plot.setLabel("bottom","Time",units="s",)
        self.plot.showGrid(x=True,y=True,alpha=0.3,)
        self.plot.setYRange(-4.0,4.0,padding=0.0)

        self.curve_meas_1 = (self.plot.plot(pen=pg.mkPen("y",width=2),name="Measured 1",))
        self.curve_ref_1 = (self.plot.plot(pen=pg.mkPen("r",width=2),name="Reference 1",))

        self.curve_meas_2 = (self.plot.plot(pen=pg.mkPen("g",width=2),name="Measured 2",))
        self.curve_ref_2 = (self.plot.plot(pen=pg.mkPen("b",width=2),name="Reference 2",))

        main_layout.addWidget(self.plot)


        self.settings_widget = SettingsWidget(self.niClient.config,"data/measurements.csv",)
        self.settings_widget.settings_changed.connect(self.apply_settings)
        self.settings_widget.status_message.connect(self.print_status)

        tabs.addTab(graph_page,"Current graph",)
        tabs.addTab(self.settings_widget,"Settings",)

        self.setCentralWidget(tabs)


    def start_control(self,):

        self.logger.info("Start button pressed")
        self.print_status("Starting controller")


        self.clear_measurements()
        self.settings_widget.apply_settings()

        manual_enabled = self.enable_manual.isChecked()
        self.niClient.set_manual_control(manual_enabled)

        if manual_enabled:
            self.niClient.update_currents(self.current_slider_1.get_value(),self.current_slider_2.get_value(),)

        self.niClient.start_control()


    def stop_control(self,):

        self.logger.info("Stop button pressed")
        self.print_status("Stopping controller")

        self.niClient.stop_control()
        self.clear_measurements()
        self.reset_sliders()

    def apply_settings(self,config,filename,):

        self.niClient.update_config(config)
        self.saverClient.start(filename)
        self.print_status(f"Settings applied; CSV: {filename}")

    def on_manual_changed(self,enabled,):

        enabled = bool(enabled)

        self.logger.info("Manual control = %s",enabled,)
        self.niClient.set_manual_control(enabled)

        if enabled:
            current_1 = self.current_slider_1.get_value()
            current_2 = self.current_slider_2.get_value()

            self.niClient.update_currents(current_1,current_2,)
            self.print_status("Manual override enabled")

        else:

            self.reset_sliders()

            self.niClient.update_currents(0.0,0.0,)
            self.print_status("Automatic sequence enabled")


    def on_current_slider_changed(self,_value,):

        if not self.enable_manual.isChecked():
            return

        current_1 = (self.current_slider_1.get_value())
        current_2 = (self.current_slider_2.get_value())

        self.niClient.update_currents(current_1,current_2,)
        self.logger.debug("Manual reference: ""%.3f A, %.3f A",current_1,current_2,)

    def update_gains(self,_value=None,):

        try:
            kp = self.kp.get_value()
            ki = self.ki.get_value()
        except (ValueError,TypeError,):
            return

        self.niClient.update_PI_coefs(kp,ki,)
        self.logger.info("PI updated: Kp=%.4f Ki=%.4f",kp,ki,)


    def update_plot(self,reference_1,measurement_1,reference_2,measurement_2,timestamp,):


        self.timestamp.append(timestamp)
        self.set_data_1.append(reference_1)
        self.meas_data_1.append(measurement_1)
        self.set_data_2.append(reference_2)
        self.meas_data_2.append(measurement_2)


        if (len(self.timestamp)> self.max_points):

            self.timestamp = (self.timestamp[-self.max_points:])
            self.set_data_1 = (self.set_data_1[-self.max_points:])
            self.meas_data_1 = (self.meas_data_1[-self.max_points:])
            self.set_data_2 = (self.set_data_2[-self.max_points:])
            self.meas_data_2 = (self.meas_data_2[-self.max_points:])

        self.current_1_view.set_value(measurement_1)
        self.current_2_view.set_value(measurement_2)

        self.curve_meas_1.setData(self.timestamp,self.meas_data_1,)
        self.curve_ref_1.setData(self.timestamp,self.set_data_1,)
        self.curve_meas_2.setData(self.timestamp,self.meas_data_2,)
        self.curve_ref_2.setData(self.timestamp,self.set_data_2,)


    def clear_measurements(self,):

        self.logger.info("Clearing measurement data")

        self.timestamp.clear()

        self.meas_data_1.clear()
        self.set_data_1.clear()

        self.meas_data_2.clear()
        self.set_data_2.clear()

        self.curve_meas_1.setData([], [])
        self.curve_ref_1.setData([], [])
        self.curve_meas_2.setData([], [])
        self.curve_ref_2.setData([], [])

        self.current_1_view.reset()
        self.current_2_view.reset()


    def reset_sliders(self,):

        self.current_slider_1.blockSignals(True)
        self.current_slider_2.blockSignals(True)

        try:
            self.current_slider_1.reset()
            self.current_slider_2.reset()

        finally:

            self.current_slider_1.blockSignals(False)
            self.current_slider_2.blockSignals(False)


    def ni_error(self,message,):

        self.logger.error("NI error: %s",message,)
        self.print_status(f"NI error: {message}")

        # GUI safe state.
        self.enable_manual.blockSignals(True)
        self.enable_manual.setChecked(False)
        self.enable_manual.blockSignals(False)

        self.reset_sliders()
        self.clear_measurements()

        self.status_label.value_field.setText("ERROR")


    def ni_status_changed(self,status,):

        status_text = "Running" if status else "Stopped"

        self.logger.info("NI status: %s",status_text,)
        self.print_status(f"NI status: {status_text}")

        self.status_label.value_field.setText(status_text)

    def saver_status_changed(self,message,):

        self.logger.info("%s",message,)
        self.print_status(message)

    def saver_error(self,message,):

        self.logger.error("%s",message,)
        self.print_status(message)

    def print_status(self,message,):

        if not hasattr(self, "software_status"):
            return

        self.software_status.appendPlainText(f"{datetime.now():%H:%M:%S} | {message}")
        scrollbar = self.software_status.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


    def closeEvent(self,event,):

        self.logger.info("Closing application")

        try:
            self.niClient.shutdown()
        except Exception:
            self.logger.exception("Error shutting down NI client")

        try:
            self.saverClient.shutdown()

        except Exception:
            self.logger.exception("Error shutting down CSV saver")

        self.reset_sliders()
        self.clear_measurements()

        event.accept()



if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())
