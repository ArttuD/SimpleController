"""CSV measurement saver running in a separate process, so it has its own GIL."""

import logging
import multiprocessing as mp
import queue as queue_module
from pathlib import Path

import numpy as np

from PyQt6.QtCore import QObject, QTimer, pyqtSignal


CSV_HEADER = "timestamp_s,reference_1_A,measurement_1_A,reference_2_A,measurement_2_A,output_1_V,output_2_V\r\n"
ROW_FORMAT = "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\r\n"


def _saver_loop(command_queue, status_queue):
    """Child process entry point: formats rows without ever taking the parent GIL."""

    file = None
    path = None

    try:
        while True:

            item = command_queue.get()

            if item is None:
                break

            kind, payload = item

            if kind == "open":

                if file is not None:
                    file.flush()
                    file.close()

                path = Path(payload)
                path.parent.mkdir(parents=True, exist_ok=True)

                file = path.open("w",newline="",encoding="utf-8",)
                file.write(CSV_HEADER)
                file.flush()

                status_queue.put(("status", f"Saving measurements: {path.name}",))

            elif kind == "batch":

                if file is None:
                    continue

                file.write("".join(ROW_FORMAT % tuple(row) for row in payload.T.tolist()))

            elif kind == "close":

                if file is not None:
                    file.flush()
                    file.close()
                    file = None
                    status_queue.put(("status", f"Measurements saved: {path.name}",))

    except Exception as exc:
        status_queue.put(("error", f"CSV saver error: {exc}",))

    finally:
        if file is not None:
            try:
                file.flush()
                file.close()
            except Exception:
                pass


class SaverClient(QObject):

    error = pyqtSignal(str)
    status_changed = pyqtSignal(str)

    def __init__(self, parent=None, queue_depth=64):

        super().__init__(parent)

        self.logger = logging.getLogger("Driver.CsvSaver")

        self.command_queue = mp.Queue(maxsize=queue_depth)
        self.status_queue = mp.Queue()

        self.process = mp.Process(target=_saver_loop,args=(self.command_queue, self.status_queue,),daemon=True,)
        self.process.start()

        self.dropped_batches = 0

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(500)
        self.poll_timer.timeout.connect(self._poll_status)
        self.poll_timer.start()

        self.logger.info("CSV saver process started (pid %s)",self.process.pid,)

    def start(self, path):

        self._send(("open", str(path),), blocking=True)

    def close(self):

        self._send(("close", None,), blocking=True)

    def save_batch(self, batch):

        # Never block the control thread: a full queue costs a log gap, not a stall.
        self._send(("batch", batch,), blocking=False)

    def save_data(self,reference_1,measurement_1,reference_2,measurement_2,timestamp,output_1=float("nan"),output_2=float("nan"),):

        column = np.array([[timestamp],[reference_1],[measurement_1],[reference_2],[measurement_2],[output_1],[output_2]],dtype=np.float64,)
        self.save_batch(column)

    def _send(self, item, blocking):

        if self.process is None:
            return

        try:
            if blocking:
                self.command_queue.put(item, timeout=1.0)
            else:
                self.command_queue.put_nowait(item)

        except queue_module.Full:
            self.dropped_batches += 1

        except Exception as exc:
            self.error.emit(f"CSV saver queue error: {exc}")

    def _poll_status(self):

        while True:

            try:
                kind, message = self.status_queue.get_nowait()
            except queue_module.Empty:
                break
            except Exception:
                break

            if kind == "error":
                self.error.emit(message)
            else:
                self.status_changed.emit(message)

    def shutdown(self):

        if self.process is None:
            return

        self.poll_timer.stop()

        self._send(("close", None,), blocking=True)
        self._send(None, blocking=True)

        self.process.join(3.0)

        if self.process.is_alive():
            self.logger.warning("CSV saver process did not exit, terminating")
            self.process.terminate()
            self.process.join(1.0)

        self.process = None
        self._poll_status()

        if self.dropped_batches:
            self.logger.warning("CSV saver dropped %d batches",self.dropped_batches,)

        self.logger.info("CSV saver process stopped")
