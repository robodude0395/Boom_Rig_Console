import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QMenu, QCheckBox, QToolTip, QFileDialog, QInputDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QPainter, QPen, QColor, QCursor
import numpy as np
from scipy.interpolate import interp1d
import json


class SplineEditor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spline Animator")
        self.resize(900, 500)

        self.padding = 50
        self.duration = 5.0  # seconds
        self.fps = 60

        # Logical coordinates: (time in seconds, value)
        self.points = [(0.0, 0.0), (1.0, 0.5), (3.0, -0.5), (5.0, 0.0)]

        self.value_min = -1.0
        self.value_max = 1.0

        self.selected_point = None
        self.hovered_point = None
        self.set_start_mode = False
        self.add_mode = False

        self.playhead_time = 0.0
        self.frame = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_playhead)

        self.play_button = QPushButton("Play", self)
        self.stop_button = QPushButton("Stop", self)
        self.restart_button = QPushButton("Restart", self)
        self.set_button = QPushButton("Set Start", self)
        self.add_checkbox = QCheckBox("Add Point Mode", self)
        self.value_label = QLabel("Value: ", self)
        self.export_button = QPushButton("Export", self)
        self.import_button = QPushButton("Import", self)

        self.export_button.clicked.connect(self.export_data)
        self.import_button.clicked.connect(self.import_data)

        self.setup_ui()
        self.setMouseTracking(True)
        self.history = []

    def export_data(self):
        options = QFileDialog.Options()
        path, _ = QFileDialog.getSaveFileName(self, "Save Spline Data", "", "JSON Files (*.json);;All Files (*)", options=options)
        if path:
            data = {
                "x_max": self.duration,
                "y_min": self.value_min,
                "y_max": self.value_max,
                "points": self.points
            }
            try:
                with open(path, 'w') as f:
                    json.dump(data, f, indent=4)
                QMessageBox.information(self, "Export Success", f"Data saved to {path}")
            except Exception as e:
                QMessageBox.critical(self, "Export Failed", str(e))

    def import_data(self):
        options = QFileDialog.Options()
        path, _ = QFileDialog.getOpenFileName(self, "Open Spline Data", "", "JSON Files (*.json);;All Files (*)", options=options)
        if path:
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                # Validate & load
                self.duration = data.get("x_max", self.duration)
                self.value_min = data.get("y_min", self.value_min)
                self.value_max = data.get("y_max", self.value_max)
                pts = data.get("points", [])
                if all(len(pt) == 2 for pt in pts):
                    self.points = [tuple(pt) for pt in pts]
                else:
                    raise ValueError("Points data corrupted or invalid")
                self.update()
                QMessageBox.information(self, "Import Success", f"Data loaded from {path}")
            except Exception as e:
                QMessageBox.critical(self, "Import Failed", str(e))

    
    def save_state(self):
        self.history.append(self.points.copy())

    def keyPressEvent(self, event):
        if event.modifiers() == Qt.ControlModifier and event.key() == Qt.Key_Z:
            if self.history:
                self.points = self.history.pop()
                self.update()

    def setup_ui(self):
        self.play_button.setGeometry(10, self.height() - 40, 60, 30)
        self.stop_button.setGeometry(80, self.height() - 40, 60, 30)
        self.restart_button.setGeometry(150, self.height() - 40, 70, 30)
        self.set_button.setGeometry(230, self.height() - 40, 80, 30)
        self.add_checkbox.setGeometry(320, self.height() - 40, 120, 30)
        self.value_label.setGeometry(460, self.height() - 40, 300, 30)

        self.play_button.clicked.connect(self.play_animation)
        self.stop_button.clicked.connect(self.stop_animation)
        self.restart_button.clicked.connect(self.restart_animation)
        self.set_button.clicked.connect(self.enable_set_start)
        self.add_checkbox.stateChanged.connect(lambda state: setattr(self, 'add_mode', state == Qt.Checked))

        self.export_button.setGeometry(600, self.height() - 40, 80, 30)
        self.import_button.setGeometry(690, self.height() - 40, 80, 30)
        self.export_button.setGeometry(600, self.height() - 40, 80, 30)
        self.import_button.setGeometry(690, self.height() - 40, 80, 30)

    def resizeEvent(self, event):
        self.setup_ui()

    def play_animation(self):
        self.timer.start(1000 // self.fps)

    def stop_animation(self):
        self.timer.stop()

    def restart_animation(self):
        self.timer.stop()
        self.frame = 0
        self.playhead_time = 0.0
        self.update()

    def enable_set_start(self):
        self.set_start_mode = True

    def update_playhead(self):
        self.frame += 1
        t = self.frame / (self.fps * self.duration)
        if t > 1.0:
            self.timer.stop()
            return
        self.playhead_time = t * self.duration
        self.update_value_label()
        self.update()

    def update_value_label(self):
        if len(self.points) < 2:
            return
        times, values = zip(*sorted(self.points))
        f = interp1d(times, values, kind='cubic', fill_value="extrapolate")
        current_value = float(f(self.playhead_time))
        self.value_label.setText(f"Value: {current_value:.2f} @ {self.playhead_time:.2f}s")

    def paintEvent(self, event):
        painter = QPainter(self)
        self.draw_grid(painter)
        self.draw_axes(painter)
        self.draw_spline(painter)
        self.draw_points(painter)
        self.draw_playhead(painter)

        if self.hovered_point:
            t, v = self.hovered_point
            QToolTip.showText(QCursor.pos(), f"({t:.2f}s, {v:.2f})")

    def draw_grid(self, painter):
        painter.save()
        painter.setPen(QPen(QColor(230, 230, 255), 1))

        w, h = self.width(), self.height()
        for i in range(0, w, 50):
            painter.drawLine(i, 0, i, h)
        for j in range(0, h, 50):
            painter.drawLine(0, j, w, j)
        painter.restore()

    def draw_axes(self, painter):
        painter.setPen(QPen(Qt.black, 2))

        # X and Y axes
        cx = self.padding
        cy = self.value_to_y(0)
        painter.drawLine(self.padding, self.padding, self.padding, self.height() - self.padding)
        painter.drawLine(self.padding, int(cy), self.width() - self.padding, int(cy))

        # X ticks (time)
        for i in range(6):
            t = i * self.duration / 5
            x = self.time_to_x(t)
            painter.drawLine(int(x), int(cy) - 5, int(x), int(cy) + 5)
            painter.drawText(int(x) - 10, int(cy) + 20, f"{t:.1f}s")

        # Y ticks (value)
        for j in range(5):
            v = self.value_max - j * (self.value_max - self.value_min) / 4
            y = self.value_to_y(v)
            painter.drawLine(self.padding - 5, int(y), self.padding + 5, int(y))
            painter.drawText(5, int(y) + 5, f"{v:.1f}")

    def draw_points(self, painter):
        painter.setPen(QPen(Qt.red, 5))
        for t, v in self.points:
            x = self.time_to_x(t)
            y = self.value_to_y(v)
            painter.drawPoint(int(x), int(y))

    def draw_spline(self, painter):
        if len(self.points) < 2:
            return
        self.points.sort()
        times, values = zip(*self.points)

        # Ensure times are strictly increasing by adding a tiny offset if needed
        for i in range(1, len(times)):
            if times[i] <= times[i - 1]:
                times = list(times)
                times[i] = times[i - 1] + 1e-5
                times = tuple(times)

        kind = 'cubic' if len(self.points) >= 4 else 'linear'
        try:
            f = interp1d(times, values, kind=kind, fill_value="extrapolate")
        except Exception as e:
            # fallback to linear if anything goes wrong
            f = interp1d(times, values, kind='linear', fill_value="extrapolate")

        painter.setPen(QPen(Qt.blue, 2))
        last_valid = False
        for px in range(self.padding, self.width() - self.padding):
            t = self.x_to_time(px)
            v = float(f(t))
            py = self.value_to_y(v)
            if self.padding <= py <= self.height() - self.padding:
                if last_valid:
                    painter.drawLine(prev_px, prev_py, px, int(py))
                prev_px = px
                prev_py = int(py)
                last_valid = True
            else:
                last_valid = False


    def draw_playhead(self, painter):
        if not self.timer.isActive() and self.playhead_time <= 0:
            return
        x = self.time_to_x(self.playhead_time)
        painter.setPen(QPen(Qt.green, 2))
        painter.drawLine(int(x), self.padding, int(x), self.height() - self.padding)

    # ---------- Coordinate Transformations ----------
    def time_to_x(self, t):
        return self.padding + (t / self.duration) * (self.width() - 2 * self.padding)

    def value_to_y(self, v):
        return self.padding + ((self.value_max - v) / (self.value_max - self.value_min)) * (self.height() - 2 * self.padding)

    def x_to_time(self, x):
        return ((x - self.padding) / (self.width() - 2 * self.padding)) * self.duration

    def y_to_value(self, y):
        return self.value_max - ((y - self.padding) / (self.height() - 2 * self.padding)) * (self.value_max - self.value_min)
    
    def set_y_limit(self, which):
        current_val = self.value_min if which == 'min' else self.value_max
        val, ok = QInputDialog.getDouble(self, f"Set Y {which.upper()}", 
                                        f"Enter new Y {which} value:", 
                                        current_val, -10000, 10000, 2)
        if ok:
            if which == 'min':
                if val >= self.value_max:
                    QMessageBox.warning(self, "Invalid Value", "Y min must be less than Y max.")
                    return
                self.value_min = val
            else:
                if val <= self.value_min:
                    QMessageBox.warning(self, "Invalid Value", "Y max must be greater than Y min.")
                    return
                self.value_max = val
            self.update()

    def set_x_limit(self):
        current_val = self.duration
        val, ok = QInputDialog.getDouble(self, "Set X Max",
                                        "Enter new X max (seconds):",
                                        current_val, 0.1, 10000, 2)
        if ok:
            if val <= 0:
                QMessageBox.warning(self, "Invalid Value", "X max must be positive and greater than 0.")
                return
            self.duration = val
            # Also adjust points to fit new duration max if needed
            self.points = [(min(t, self.duration), v) for (t, v) in self.points]
            self.update()


    # ---------- Interaction ----------
    def mousePressEvent(self, event):
        x, y = event.x(), event.y()
        # Detect right click near Y axis labels (left padding area)
        if event.button() == Qt.RightButton:
            if x < self.padding:
                # Near Y axis labels
                menu = QMenu(self)
                set_ymin = menu.addAction("Set Y Min")
                set_ymax = menu.addAction("Set Y Max")
                set_xmax = menu.addAction("Set X Max")
                action = menu.exec_(event.globalPos())
                if action == set_ymin:
                    self.set_y_limit('min')
                elif action == set_ymax:
                    self.set_y_limit('max')
                elif action == set_xmax:
                    self.set_x_limit()
                return

        x, y = event.x(), event.y()
        t, v = self.x_to_time(x), self.y_to_value(y)

        if self.set_start_mode:
            self.playhead_time = max(0.0, min(self.duration, t))
            self.frame = int((self.playhead_time / self.duration) * self.fps * self.duration)
            self.set_start_mode = False
            self.update()
            return
        
        if self.add_mode:
                self.save_state()
                self.points.append((t, v))
                self.update()

        if event.button() == Qt.RightButton:
            for i, (pt, pv) in enumerate(self.points):
                px, py = self.time_to_x(pt), self.value_to_y(pv)
                if abs(px - x) < 8 and abs(py - y) < 8:
                    menu = QMenu(self)
                    action = menu.addAction("Delete Point")
                    action.triggered.connect(lambda _, idx=i: self.delete_point(idx))
                    menu.exec_(event.globalPos())
                    return

        elif event.button() == Qt.LeftButton:
            for i, (pt, pv) in enumerate(self.points):
                px, py = self.time_to_x(pt), self.value_to_y(pv)
                if abs(px - x) < 8 and abs(py - y) < 8:
                    self.selected_point = i
                    return

            if self.add_mode:
                self.points.append((t, v))
                self.update()

    def mouseMoveEvent(self, event):
        x, y = event.x(), event.y()
        t, v = self.x_to_time(x), self.y_to_value(y)
        self.hovered_point = None

        for pt, pv in self.points:
            px, py = self.time_to_x(pt), self.value_to_y(pv)
            if abs(px - x) < 8 and abs(py - y) < 8:
                self.hovered_point = (pt, pv)

        if self.selected_point is not None:
            t = max(0.0, min(self.duration, t))
            v = max(self.value_min, min(self.value_max, v))
            self.points[self.selected_point] = (t, v)
            self.hovered_point = (t, v)

        self.update()

    def mouseReleaseEvent(self, event):
        self.selected_point = None

    def delete_point(self, index):
        if 0 <= index < len(self.points):
            del self.points[index]
            self.update()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SplineEditor()
    window.show()
    sys.exit(app.exec_())
