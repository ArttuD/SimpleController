
import gc
import logging
import math
import sys
import threading
import time
from dataclasses import dataclass, replace

import numpy as np
import nidaqmx

from PyQt6.QtCore import (
    QObject,
    Qt,
    QThread,
    pyqtSignal,
    pyqtSlot,
)

from nidaqmx.constants import (
    AcquisitionType,
    ReadRelativeTo,
    TaskMode,
)

from nidaqmx.stream_readers import (
    AnalogMultiChannelReader,
)

from nidaqmx.stream_writers import (
    AnalogMultiChannelWriter,
)


# Windows schedules on a 15.6 ms tick by default, and a thread waiting for the GIL
# rounds up to it. timeBeginPeriod(1) drops that to 1 ms for the whole process.
if sys.platform == "win32":
    import ctypes
    _winmm = ctypes.WinDLL("winmm")
    _kernel32 = ctypes.WinDLL("kernel32")
else:
    _winmm = None
    _kernel32 = None

HIGH_PRIORITY_CLASS = 0x00000080
NORMAL_PRIORITY_CLASS = 0x00000020


@dataclass(frozen=True, slots=True)
class ControlConfig:
    """All runtime tuning values used by the NI control worker."""

    device_name: str = "Dev3"
    ai_channel: str = "ai0:1"
    ao_channel: str = "ao0:1"

    # Acquisition runs fast for data fidelity; control updates once per batch.
    # Raise sample_rate_AI and read_batch_size_AI together to keep the control rate fixed.
    sample_rate_AI: float = 10000.0 # Hardware AI sample rate
    read_batch_size_AI: int = 10

    sequence_type: str = "Oscillatory test"

    # 50 s of AI backlog absorbs transient Windows stalls without overflowing.
    ai_buffer_size: int = 100000

    r_shunt_1: float = 0.12
    r_shunt_2: float = 0.12
    min_voltage: float = -10.0
    max_voltage: float = 10.0

    kp: float = 20.0
    ki: float = 10.0
    kff: float = 4.0 # Feedforward [V/A], ideally 1 / amplifier gain
    measurement_filter_hz: float = 0.0 # Low-pass on the controlled current, 0 = off

    peak_amplitude_POS: float = 2.0
    peak_amplitude_NEG: float = -0.5
    peak_width: float = 1.0
    peak_interval: float = 2.0
    peak_number: int = 3

    pre_magnetization_pause: float = 0.5
    magnetization_period: float = 5.0
    magnetization_amplitude: float = 0.75

    sine_frequency: float = 0.05 # Sine frequency
    sine_offset: float = 0.75
    sine_amplitude: float = 1.25
    sine_periods: float = 1.5 # Number of sine periods
    post_sequence_pause: float = 1.0
    pulse_amplitude: float = 2.0
    pulse_period: float = 2.0
    pulse_width: float = 0.5
    pulse_cycles: int = 10
    sawtooth_min: float = 0.0
    sawtooth_max: float = 2.0
    sawtooth_period: float = 10.0
    sawtooth_cycles: int = 2
    # Plot redraws run on the GUI thread and contend for the GIL with the DAQ callback.
    visualization_rate: float = 10.0
    log_emit_interval: float = 0.2 # Seconds of samples buffered per saver emit

    @property
    def control_dt(self):
        return 1.0 / self.sample_rate_AI    



class PIController:

    __slots__ = ("kp","ki","output_min","output_max","integral",)

    def __init__(self,kp=20.0,ki=10.0,output_min=-10.0,output_max=10.0,):

        self.kp = float(kp)
        self.ki = float(ki)

        self.output_min = float(output_min)
        self.output_max = float(output_max)

        self.integral = 0.0

    def set_coefs(self, kp, ki):

        self.kp = float(kp)
        self.ki = float(ki)

    def reset(self):

        self.integral = 0.0

    def process(self,reference,measurement,dt,feedforward=0.0,):

        error = reference - measurement

        candidate_integral = (self.integral+ error * dt)
        raw_output = (feedforward + self.kp * error+ self.ki * candidate_integral)

        if raw_output > self.output_max:

            output = self.output_max

            # Integrate only if error would reduce saturation (Anti windup).
            if error < 0.0:
                self.integral = candidate_integral

        elif raw_output < self.output_min:
            output = self.output_min

            if error > 0.0:
                self.integral = candidate_integral

        else:
            self.integral = candidate_integral
            output = raw_output

        return float(output)


class NiClientWorker(QObject):

    error = pyqtSignal(str)
    status_changed = pyqtSignal(bool)
    data_updated = pyqtSignal(float,float,float,float,float,)
    samples_updated = pyqtSignal(object)
    finished = pyqtSignal()
    finalize_signal = pyqtSignal()

    def __init__(self, config=None, parent=None):

        super().__init__(parent)

        # Queued so task teardown never runs inside the DAQmx callback thread.
        self.finalize_signal.connect(self._finalize,type=Qt.ConnectionType.QueuedConnection,)

        self.config = config or ControlConfig()

        self.manual_control = False
        self.current_1_feed = 0.0
        self.current_2_feed = 0.0
        self.kp = self.config.kp
        self.ki = self.config.ki

        self.logger = logging.getLogger("Driver.NiClient")

        self.running = False
        self.stop_requested = False
        self.manual_control = False

        self.current_1_feed = 0.0
        self.current_2_feed = 0.0

        self.sequence_index = 0
        self.reset_sequence = False
        self.control_start_time = 0.0

        self.pi_1 = PIController(self.config.kp,self.config.ki,self.config.min_voltage,self.config.max_voltage,)
        self.pi_2 = PIController(self.config.kp,self.config.ki,self.config.min_voltage,self.config.max_voltage,)

        self.ai_task = None
        self.ao_task = None
        self.ai_reader = None

        self.ai_data = np.zeros((2, self.config.read_batch_size_AI),dtype=np.float64,)
        self.ao_data = np.zeros(2,dtype=np.float64,)
        self.output_lock = threading.Lock()

        # Cached so the callback never touches a property or a dataclass field.
        self.batch_size = self.config.read_batch_size_AI
        self.control_dt = self.config.control_dt
        self.control_period = self.batch_size * self.control_dt

        self.inv_r_shunt_1 = 1.0 / self.config.r_shunt_1
        self.inv_r_shunt_2 = 1.0 / self.config.r_shunt_2

        self.kff = self.config.kff

        filter_hz = self.config.measurement_filter_hz
        self.filter_alpha = 1.0 - math.exp(-2.0 * math.pi * filter_hz * self.control_period) if filter_hz > 0.0 else 1.0
        self.filtered_1 = 0.0
        self.filtered_2 = 0.0

        self.perf_counter = time.perf_counter
        self.sample_offsets = np.arange(self.batch_size,dtype=np.float64,) * self.control_dt

        self.sequence_list = None
        self.sequence_list_2 = None
        self.sequence_length = 0

        self.finalizing = False
        self.previous_switch_interval = None
        self.timer_resolution_raised = False
        self.gc_was_enabled = False

        # Logs every acquired sample, not just one row per control update.
        batches_per_emit = max(1,int(round(self.config.log_emit_interval/ self.control_period)),)
        self.log_buffer = np.zeros((7, batches_per_emit * self.batch_size),dtype=np.float64,)
        self.log_fill = 0

        self.ao_writer = None
        self.sequence = None
        self.sequence_2 = None

        self.iteration_count = 0
        self.next_visualization_time = 0.0

        self.last_callback_time = 0.0
        self.max_callback_interval = 0.0

        self.logger.info("NI worker initialized: %.0f Hz control",self.config.sample_rate_AI / self.batch_size,)


    def _build_sequence(self):

        config = self.config
        rate = config.sample_rate_AI

        if config.sequence_type == "Pulse":
            self.sequence = self._build_pulse_sequence(config, rate)
            self.sequence_2 = self.sequence
            self.sequence_index = 0
            self._log_sequence("pulse")
            return

        if config.sequence_type == "Sawtooth":
            self.sequence = self._build_sawtooth_sequence(config, rate)
            self.sequence_2 = self.sequence
            self.sequence_index = 0
            self._log_sequence("sawtooth")
            return

        peak_cycle = (config.peak_interval+config.peak_width)

        peak_cycle_samples = int(round(peak_cycle * rate))
        peak_interval_samples = int(round(config.peak_interval * rate))
        peak_width_samples = int(round(config.peak_width * rate))

        peak_cycle_array_1 = np.zeros(peak_cycle_samples,dtype=np.float64,)
        peak_cycle_array_2 = np.zeros(peak_cycle_samples,dtype=np.float64,)

        peak_start = peak_interval_samples
        peak_end = min(peak_start + peak_width_samples,peak_cycle_samples,)

        peak_cycle_array_1[peak_start:peak_end] = config.peak_amplitude_POS
        peak_cycle_array_2[peak_start:peak_end] = config.peak_amplitude_NEG
        pre_peak = np.tile(peak_cycle_array_1,config.peak_number,)
        pre_peak_2 = np.tile(peak_cycle_array_2,config.peak_number,)

        pre_magnetization_pause_samples = int(round(config.pre_magnetization_pause* rate))
        pre_magnetization_pause = np.zeros(pre_magnetization_pause_samples,dtype=np.float64,)

        magnetization_samples = int(round(config.magnetization_period * rate))
        magnetization = np.full(magnetization_samples,config.magnetization_amplitude,dtype=np.float64,)


        sine_duration = (config.sine_periods/ config.sine_frequency)
        sine_samples = int(round(sine_duration * rate))
        sine_t = (np.arange(sine_samples,dtype=np.float64,)/ rate)
        sine_wave = (config.sine_amplitude* np.sin(2.0* np.pi* config.sine_frequency* sine_t))

        # Opposed swings around a common offset: coil 1 rises while coil 2 falls.
        sine_1 = config.sine_offset + sine_wave
        sine_2 = config.sine_offset - sine_wave

        post_peak = np.tile(peak_cycle_array_1,config.peak_number,)
        post_peak_2 = np.tile(peak_cycle_array_2,config.peak_number,)
        post_sequence_pause_samples = int(round(config.post_sequence_pause* rate))
        post_sequence_pause = np.zeros(post_sequence_pause_samples,dtype=np.float64,)

        common_head = (pre_peak,pre_magnetization_pause,magnetization,)
        common_head_2 = (pre_peak_2,pre_magnetization_pause,magnetization,)
        common_tail = (post_peak,post_sequence_pause,)
        common_tail_2 = (post_peak_2,post_sequence_pause,)

        self.sequence = np.concatenate(common_head + (sine_1,) + common_tail)
        self.sequence_2 = np.concatenate(common_head_2 + (sine_2,) + common_tail_2)

        self.sequence_index = 0

        self.logger.info("Sequence precomputed: %d samples, %.3f s",len(self.sequence),len(self.sequence) / rate,)


    def _build_pulse_sequence(self, config, rate):

        period_samples = max(1,int(round(config.pulse_period * rate)),)
        width_samples = min(period_samples,max(1, int(round(config.pulse_width * rate))),)

        pulse = np.zeros(period_samples, dtype=np.float64)
        pulse[:width_samples] = config.pulse_amplitude

        pause = np.zeros(max(0, int(round(config.post_sequence_pause * rate))),dtype=np.float64,)

        return np.concatenate((np.tile(pulse, config.pulse_cycles),pause,))

    def _build_sawtooth_sequence(self, config, rate):

        period_samples = max(1,int(round(config.sawtooth_period * rate)),)
        sawtooth = np.linspace(config.sawtooth_min,config.sawtooth_max,period_samples,endpoint=False,dtype=np.float64,)

        pause = np.zeros(max(0, int(round(config.post_sequence_pause * rate))),dtype=np.float64,)

        return np.concatenate((np.tile(sawtooth, config.sawtooth_cycles),pause,))

    def _log_sequence(self, name):

        self.logger.info("Sequence precomputed: %d samples, %.3f s",len(self.sequence),len(self.sequence) / self.config.sample_rate_AI,)
        self.logger.info("Sequence mode: %s",name,)


    def _create_tasks(self):

        self.logger.info("Creating NI tasks")
        self.ai_task = nidaqmx.Task("Driver_AI")

        self.ai_task.ai_channels.add_ai_voltage_chan(f"{self.config.device_name}/{self.config.ai_channel}")
        self.ai_task.timing.cfg_samp_clk_timing(rate=self.config.sample_rate_AI,sample_mode=AcquisitionType.CONTINUOUS,samps_per_chan=self.config.ai_buffer_size,)
        self.ai_task.in_stream.relative_to = (ReadRelativeTo.CURRENT_READ_POSITION)
        self.ai_task.in_stream.offset = 0

        # On-demand AO: the output is applied as soon as it is written, with no
        # queue ahead of it and no underflow failure mode.
        self.ao_task = nidaqmx.Task("Driver_AO")
        self.ao_task.ao_channels.add_ao_voltage_chan(f"{self.config.device_name}/{self.config.ao_channel}",min_val=self.config.min_voltage,max_val=self.config.max_voltage,)

        # Commit and start up front, otherwise every write pays a task transition.
        self.ao_task.control(TaskMode.TASK_COMMIT)
        self.ao_task.start()

        self.ai_reader = AnalogMultiChannelReader(self.ai_task.in_stream)
        self.ao_writer = AnalogMultiChannelWriter(self.ao_task.out_stream,auto_start=False,)

        self.ai_task.register_every_n_samples_acquired_into_buffer_event(self.config.read_batch_size_AI,self._reading_callback,)

        self.logger.info("AI configured: %.0f Hz continuous, EveryN callback every %d samples",self.config.sample_rate_AI,self.config.read_batch_size_AI,)
        self.logger.info("AO configured: on-demand, control period %.3f ms",self.control_period * 1000.0,)



    @pyqtSlot()
    def start(self):

        if self.running:

            self.logger.warning("Start requested while already running")
            return

        try:

            self.logger.info("Starting NI control")

            self._cleanup_tasks()
            self.stop_requested = False

            self._build_sequence()

            # Python list indexing keeps references off numpy scalars in the hot loop.
            self.sequence_list = self.sequence.tolist()
            self.sequence_list_2 = self.sequence_2.tolist()
            self.sequence_length = len(self.sequence_list)

            self.pi_1.reset()
            self.pi_2.reset()

            self.sequence_index = 0
            self.reset_sequence = False
            self.iteration_count = 0
            self.log_fill = 0
            self.finalizing = False

            self._create_tasks()

            self.ao_data.fill(0.0)
            self.ao_writer.write_one_sample(self.ao_data,timeout=0.050,)

            self.running = True
            self.control_start_time = time.perf_counter()
            self.next_visualization_time = (self.control_start_time+ 1.0 / self.config.visualization_rate)

            self.last_callback_time = 0.0
            self.max_callback_interval = 0.0

            self.previous_switch_interval = sys.getswitchinterval()
            sys.setswitchinterval(0.00005)

            self._enter_realtime_mode()

            self.ai_task.start()

            self.status_changed.emit(True)
            self.logger.info("NI control started")

        except Exception as exc:

            self.logger.exception("Failed to start NI control")

            self.running = False
            self._restore_scheduler_resolution()
            self.status_changed.emit(False)
            self.error.emit(f"NI start error: {exc}")
            self._cleanup_tasks()

    def _reading_callback(self, task_handle, event_type, number_of_samples, callback_data):
        """Runs on a DAQmx driver thread once per batch_size acquired samples."""

        if not self.running or self.finalizing:
            return 0

        try:

            perf_counter = self.perf_counter
            started = perf_counter()

            last_callback_time = self.last_callback_time
            self.last_callback_time = started

            if last_callback_time:
                interval = started - last_callback_time
                if interval > self.max_callback_interval:
                    self.max_callback_interval = interval

            self.ai_reader.read_many_sample(self.ai_data, number_of_samples_per_channel=self.batch_size,timeout=0.010,)

            means = self.ai_data.mean(axis=1).tolist()
            inv_r_shunt_1 = self.inv_r_shunt_1
            inv_r_shunt_2 = self.inv_r_shunt_2
            measurement_1 = means[0] * inv_r_shunt_1
            measurement_2 = means[1] * inv_r_shunt_2

            filter_alpha = self.filter_alpha
            self.filtered_1 += filter_alpha * (measurement_1 - self.filtered_1)
            self.filtered_2 += filter_alpha * (measurement_2 - self.filtered_2)

            if self.manual_control:
                reference_1 = self.current_1_feed
                reference_2 = self.current_2_feed

                log_reference_1 = reference_1
                log_reference_2 = reference_2
            else:
                if self.reset_sequence:
                    self.reset_sequence = False
                    self.sequence_index = 0

                start_index = self.sequence_index
                sequence_index = start_index + self.batch_size - 1

                if sequence_index >= self.sequence_length:
                    self._request_finalize()
                    return 0

                # Control uses the newest reference in the window; the log keeps them all.
                reference_1 = self.sequence_list[sequence_index]
                reference_2 = self.sequence_list_2[sequence_index]
                log_reference_1 = self.sequence[start_index:start_index + self.batch_size]
                log_reference_2 = self.sequence_2[start_index:start_index + self.batch_size]

                self.sequence_index = start_index + self.batch_size

            control_period = self.control_period

            ao_data = self.ao_data
            kff = self.kff
            ao_data[0] = self.pi_1.process(reference_1,self.filtered_1,control_period,kff * reference_1,)
            ao_data[1] = self.pi_2.process(reference_2,self.filtered_2,control_period,kff * reference_2,)

            # Shared with _write_zero so a stop can never be followed by a late non-zero write.
            
            with self.output_lock:
                if self.finalizing:
                    return 0
                
                self.ao_writer.write_one_sample(ao_data,timeout=0.010,)
                
            now = perf_counter()

            fill = self.log_fill
            end = fill + self.batch_size

            log_buffer = self.log_buffer
            ai_data = self.ai_data

            log_buffer[0, fill:end] = self.iteration_count * self.control_dt + self.sample_offsets
            log_buffer[1, fill:end] = log_reference_1
            log_buffer[2, fill:end] = ai_data[0] * inv_r_shunt_1
            log_buffer[3, fill:end] = log_reference_2
            log_buffer[4, fill:end] = ai_data[1] * inv_r_shunt_2
            log_buffer[5, fill:end] = ao_data[0]
            log_buffer[6, fill:end] = ao_data[1]
            self.log_fill = end

            self.iteration_count += self.batch_size

            if self.log_fill >= log_buffer.shape[1]:
                self._flush_log()
            if now >= self.next_visualization_time:
                self.data_updated.emit(reference_1,measurement_1,reference_2,measurement_2,self.iteration_count * self.control_dt,)
                self.next_visualization_time = now + 1.0 / self.config.visualization_rate

        except Exception as exc:

            self.logger.exception("NI control callback error")
            self.error.emit(f"NI control callback error: {exc}")
            self._request_finalize()

        return 0

    def _request_finalize(self):

        if self.finalizing:
            return

        self.finalizing = True
        self.running = False
        self.finalize_signal.emit()

    def _enter_realtime_mode(self):
        """A gen-2 GC pass or a 15.6 ms scheduler tick both stall the callback."""

        self.gc_was_enabled = gc.isenabled()
        gc.collect()
        gc.freeze()
        gc.disable()

        if _winmm is None:
            return

        try:
            if _winmm.timeBeginPeriod(1) == 0:
                self.timer_resolution_raised = True

            _kernel32.SetPriorityClass(_kernel32.GetCurrentProcess(),HIGH_PRIORITY_CLASS,)
            self.logger.info("Realtime mode: GC off, 1 ms timer, high priority")

        except Exception:
            self.logger.exception("Failed to enter realtime mode")

    def _restore_scheduler_resolution(self):

        if self.previous_switch_interval is not None:
            sys.setswitchinterval(self.previous_switch_interval)
            self.previous_switch_interval = None

        if self.gc_was_enabled:
            gc.enable()
            self.gc_was_enabled = False

        gc.unfreeze()

        if _winmm is None:
            return

        try:
            _kernel32.SetPriorityClass(_kernel32.GetCurrentProcess(),NORMAL_PRIORITY_CLASS,)

            if self.timer_resolution_raised:
                _winmm.timeEndPeriod(1)
                self.timer_resolution_raised = False

        except Exception:
            self.logger.exception("Failed to restore scheduler resolution")
            
    @pyqtSlot()
    def _finalize(self):

        self._restore_scheduler_resolution()
        self._write_zero()
        self._flush_log()
        self._log_final_performance()

        self._cleanup_tasks()

        self.status_changed.emit(False)
        self.logger.info("NI control stopped")
        self.finished.emit()

    def _log_final_performance(self):

        if self.max_callback_interval <= 0.0:
            self.logger.info("Callback interval: no intervals recorded")
            return

        self.logger.info("Callback interval: worst %.3f ms (nominal %.3f ms)",self.max_callback_interval * 1000.0,self.control_period * 1000.0,)

    def _flush_log(self):

        if self.log_fill <= 0:
            return

        self.samples_updated.emit(self.log_buffer[:, :self.log_fill].copy())
        self.log_fill = 0

    def _write_zero(self):

        if self.ao_writer is None:

            return

        try:
            # Own array: the callback may still be filling ao_data on its thread.
            with self.output_lock:
                self.ao_writer.write_one_sample(np.zeros(2,dtype=np.float64,),timeout=0.050,)

        except Exception:

            self.logger.exception("Failed to write zero output")



    @pyqtSlot()
    def request_stop(self):


        if self.running:
            self.logger.info("Stopping NI control")
            self.stop_requested = True
            self._request_finalize()

    def _reset_runtime_state(self):

        self.pi_1.reset()
        self.pi_2.reset()
        self.sequence_index = 0
        self.control_start_time = 0.0
        self.filtered_1 = 0.0
        self.filtered_2 = 0.0


    def _cleanup_tasks(self):


        if self.ai_task is not None:

            try:
                self.ai_task.stop()
            except Exception:
                pass

            try:
                self.ai_task.close()
            except Exception:
                pass


        if self.ao_task is not None:

            try:
                self.ao_task.stop()
            except Exception:
                pass

            try:
                self.ao_task.close()
            except Exception:
                pass

        self.ai_task = None
        self.ao_task = None
        self.ai_reader = None
        self.ao_writer = None
        self.running = False
        self.stop_requested = False
        self._reset_runtime_state()


    @pyqtSlot(bool)
    def set_manual_control(self,enabled,):

        self.manual_control = bool(enabled)
        self.logger.info("Manual control: %s",self.manual_control,)

        if self.manual_control:
            self.pi_1.reset()
            self.pi_2.reset()
        else:

            self.reset_sequence = True



    @pyqtSlot(float, float)
    def set_currents_feed(self,current_1,current_2,):

        self.current_1_feed = float(current_1)
        self.current_2_feed = float(current_2)

    @pyqtSlot(float, float)
    def set_PI_coefs(self,kp,ki,):

        self.pi_1.set_coefs(kp,ki,)

        self.pi_2.set_coefs(kp,ki,)

        self.logger.info("PI coefficients updated: Kp=%.4f Ki=%.4f",kp,ki,)

    @pyqtSlot(float)
    def set_r_shunt_1(self,r_shunt,):

        self.inv_r_shunt_1 = 1.0 / float(r_shunt)
        self.logger.info("Shunt 1 resistance updated: %.6f ohm",r_shunt,)

    def set_r_shunt_2(self,r_shunt,):

        self.inv_r_shunt_2 = 1.0 / float(r_shunt)
        self.logger.info("Shunt 2 resistance updated: %.6f ohm",r_shunt,)

    @pyqtSlot(float)
    def set_kff(self,kff,):

        self.kff = float(kff)
        self.logger.info("Feedforward updated: Kff=%.4f V/A",kff,)



class NiClient(QObject):


    start_signal = pyqtSignal()
    stop_signal = pyqtSignal()

    manual_control_signal = pyqtSignal(bool)
    currents_feed_signal = pyqtSignal(float,float,)
    PI_coefs_signal = pyqtSignal(float,float,)
    r_shunt_1_signal = pyqtSignal(float)
    r_shunt_2_signal = pyqtSignal(float)
    kff_signal = pyqtSignal(float)

    error = pyqtSignal(str)
    status_changed = pyqtSignal(bool)
    data_updated = pyqtSignal(float,float,float,float,float,)
    samples_updated = pyqtSignal(object)

    def __init__(self,parent=None,config=None,
    ):

        super().__init__(parent)

        self.logger = logging.getLogger(
            "Driver.NiClient"
        )

        self.config = config or ControlConfig()

        self.manual_control = False
        self.current_1_feed = 0.0
        self.current_2_feed = 0.0
        self.kp = self.config.kp
        self.ki = self.config.ki
        self.kff = self.config.kff

        self.worker = None
        self.thread = None

        self._create_worker()


    def _create_worker(self):


        if self._thread_is_running():

            return

        self.worker = NiClientWorker(config=self.config,)
        self.thread = QThread()
        self.worker.moveToThread(self.thread)

        self.start_signal.connect(
            self.worker.start,
        )

        self.stop_signal.connect(self.worker.request_stop,type=Qt.ConnectionType.DirectConnection,)
        self.manual_control_signal.connect(self.worker.set_manual_control,type=Qt.ConnectionType.DirectConnection,)
        self.currents_feed_signal.connect(self.worker.set_currents_feed,type=Qt.ConnectionType.DirectConnection,)
        self.PI_coefs_signal.connect(self.worker.set_PI_coefs,type=Qt.ConnectionType.DirectConnection,)
        self.r_shunt_1_signal.connect(self.worker.set_r_shunt_1,type=Qt.ConnectionType.DirectConnection,)
        self.r_shunt_2_signal.connect(self.worker.set_r_shunt_2,type=Qt.ConnectionType.DirectConnection,)
        self.kff_signal.connect(self.worker.set_kff,type=Qt.ConnectionType.DirectConnection,)


        self.worker.error.connect(self.error,)
        self.worker.status_changed.connect(self.status_changed,)
        self.worker.data_updated.connect(self.data_updated,)
        self.worker.samples_updated.connect(self.samples_updated,)

        self.worker.finished.connect(self._worker_finished,)
        self.thread.finished.connect(self._thread_finished,)
        # Direct: shutdown() blocks the GUI thread in wait(), so a queued quit would never run.
        self.worker.finished.connect(self.thread.quit,type=Qt.ConnectionType.DirectConnection,)
        self.worker.finished.connect(self.worker.deleteLater,)
        self.thread.finished.connect(self.thread.deleteLater,)
        self.thread.start()

        self.logger.info("NI worker thread started")


    @pyqtSlot()
    def _worker_finished(self):

        self.logger.info("NI worker finished")

    @pyqtSlot()
    def _thread_finished(self):

        # A replaced worker's thread can finish after its successor was created.
        if self.sender() is not self.thread:
            return

        self.logger.info("NI worker thread finished")

        self.worker = None
        self.thread = None

    def _thread_is_running(self):

        if self.thread is None:
            return False

        try:
            return self.thread.isRunning()
        except RuntimeError:
            self.worker = None
            self.thread = None
            return False


    def start_control(self):

        self.logger.info("Start button pressed")

        if (self.worker is None or self.thread is None or not self._thread_is_running()):

            self.logger.info("Recreating NI worker")
            self._create_worker()


        self.worker.set_manual_control(self.manual_control)
        self.worker.set_currents_feed(self.current_1_feed,self.current_2_feed,)
        self.worker.set_PI_coefs(self.kp,self.ki,)
        self.worker.set_kff(self.kff)

        self.start_signal.emit()

    def update_config(self, config):

        was_running = (self.worker is not None and self.worker.running)

        # The worker caches the config at construction, so it must always be replaced.
        self.shutdown()

        self.config = replace(config, kp=self.kp, ki=self.ki, kff=self.kff)
        self._create_worker()

        if was_running:
            self.start_control()


    def stop_control(self):

        self.logger.info(
            "Stop button pressed"
        )

        if self.worker is None:

            return

        self.stop_signal.emit()


    def set_manual_control(self,enabled,):

        self.manual_control = bool(enabled)
        self.manual_control_signal.emit(self.manual_control)


    def update_currents(self,current_1,current_2,):

        self.current_1_feed = float(current_1)
        self.current_2_feed = float(current_2)

        self.currents_feed_signal.emit(self.current_1_feed,self.current_2_feed,)

    def update_PI_coefs(self,kp,ki,):

        self.kp = float(kp)
        self.ki = float(ki)
        self.config = replace(self.config, kp=self.kp, ki=self.ki)

        self.PI_coefs_signal.emit(self.kp,self.ki,)

    def update_r_shunt_1(self,r_shunt,):

        r_shunt = float(r_shunt)
        self.config = replace(self.config, r_shunt_1=r_shunt)

        self.r_shunt_1_signal.emit(r_shunt)

    def update_r_shunt_2(self,r_shunt,):

        r_shunt = float(r_shunt)
        self.config = replace(self.config, r_shunt_2=r_shunt)

        self.r_shunt_2_signal.emit(r_shunt)

    def update_kff(self,kff,):

        self.kff = float(kff)
        self.config = replace(self.config, kff=self.kff)

        self.kff_signal.emit(self.kff)

    def _disconnect_worker(self):

        for signal in (self.start_signal,self.stop_signal,self.manual_control_signal,self.currents_feed_signal,self.PI_coefs_signal,self.r_shunt_1_signal,self.r_shunt_2_signal,self.kff_signal,):
            try:
                signal.disconnect()
            except TypeError:
                pass

    def shutdown(self):

        self.logger.info("NI shutdown requested")

        worker = self.worker
        thread = self.thread

        if thread is not None and self._thread_is_running():

            if worker is not None and worker.running:
                # _finalize zeroes the outputs, closes the tasks and then quits the thread.
                self.stop_signal.emit()
            else:
                thread.quit()

            if not thread.wait(3000):
                self.logger.warning("NI worker thread did not stop within 3 s")

        self._disconnect_worker()

        self.worker = None
        self.thread = None
