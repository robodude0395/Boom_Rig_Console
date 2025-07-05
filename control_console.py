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
        self.duration = 5.0
        self.fps = 60

        self.selected_handle = None  # (point_index, 'in' or 'out')

        # Each point: (time, value, handle_out, handle_in)
        # Only time and value to start with:
        self.points = [
            (0.0, 0.0),
            (1.0, 0.5),
            (3.0, -0.5),
            (5.0, 0.0),
        ]

        # Compute handles and update points with handles
        handles = self.compute_handles([(pt[0], pt[1]) for pt in self.points])
        self.points = [
            (pt[0], pt[1], handles[i][0], handles[i][1]) for i, pt in enumerate(self.points)
        ]

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

    def compute_handles(self, points, tension=0.3):
        handles = []
        n = len(points)
        for i in range(n):
            p0 = points[i-1][:2] if i > 0 else points[i][:2]
            p1 = points[i][:2]
            p2 = points[i+1][:2] if i < n-1 else points[i][:2]

            dx_out = (p2[0] - p0[0]) * tension
            dy_out = (p2[1] - p0[1]) * tension

            handle_out = (p1[0] + dx_out / 3, p1[1] + dy_out / 3)
            handle_in = (p1[0] - dx_out / 3, p1[1] - dy_out / 3)

            handles.append((handle_out, handle_in))
        return handles

    def cubic_bezier(self, t, P0, P1, P2, P3):
        return (
            (1 - t)**3 * P0[0] + 3 * (1 - t)**2 * t * P1[0] + 3 * (1 - t) * t**2 * P2[0] + t**3 * P3[0],
            (1 - t)**3 * P0[1] + 3 * (1 - t)**2 * t * P1[1] + 3 * (1 - t) * t**2 * P2[1] + t**3 * P3[1]
        )

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
                self.duration = data.get("x_max", self.duration)
                self.value_min = data.get("y_min", self.value_min)
                self.value_max = data.get("y_max", self.value_max)
                pts = data.get("points", [])
                if all(len(pt) >= 2 for pt in pts):
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
        self.export_button.setGeometry(650, self.height() - 40, 80, 30)
        self.import_button.setGeometry(740, self.height() - 40, 80, 30)

        self.play_button.clicked.connect(self.play_animation)
        self.stop_button.clicked.connect(self.stop_animation)
        self.restart_button.clicked.connect(self.restart_animation)
        self.set_button.clicked.connect(self.enable_set_start)
        self.add_checkbox.stateChanged.connect(lambda state: setattr(self, 'add_mode', state == Qt.Checked))

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
        current_value = self.evaluate_spline_value_at_time(self.playhead_time, self.points)
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
        cx = self.padding
        cy = self.value_to_y(0)
        painter.drawLine(self.padding, self.padding, self.padding, self.height() - self.padding)
        painter.drawLine(self.padding, int(cy), self.width() - self.padding, int(cy))
        for i in range(6):
            t = i * self.duration / 5
            x = self.time_to_x(t)
            painter.drawLine(int(x), int(cy) - 5, int(x), int(cy) + 5)
            painter.drawText(int(x) - 10, int(cy) + 20, f"{t:.1f}s")
        for j in range(5):
            v = self.value_max - j * (self.value_max - self.value_min) / 4
            y = self.value_to_y(v)
            painter.drawLine(self.padding - 5, int(y), self.padding + 5, int(y))
            painter.drawText(5, int(y) + 5, f"{v:.1f}")

    def draw_points(self, painter):
        painter.setPen(QPen(Qt.red, 5))
        radius = 4

        for t, v, handle_out, handle_in in self.points:
            x = self.time_to_x(t)
            y = self.value_to_y(v)
            painter.drawPoint(int(x), int(y))
            painter.drawEllipse(int(x - radius), int(y - radius), 2 * radius, 2 * radius)

            # Draw handle_out line and point
            if handle_out:
                hx = self.time_to_x(handle_out[0])
                hy = self.value_to_y(handle_out[1])
                painter.setPen(QPen(Qt.darkGray, 1))
                painter.drawLine(int(x), int(y), int(hx), int(hy))
                painter.setPen(QPen(Qt.darkBlue, 2))
                painter.drawEllipse(int(hx - radius), int(hy - radius), 2 * radius, 2 * radius)

            # Draw handle_in line and point
            if handle_in:
                hx = self.time_to_x(handle_in[0])
                hy = self.value_to_y(handle_in[1])
                painter.setPen(QPen(Qt.darkGray, 1))
                painter.drawLine(int(x), int(y), int(hx), int(hy))
                painter.setPen(QPen(Qt.darkGreen, 2))
                painter.drawEllipse(int(hx - radius), int(hy - radius), 2 * radius, 2 * radius)


    def draw_spline(self, painter):
        if len(self.points) < 2:
            return

        painter.setPen(QPen(Qt.blue, 2))

        for i in range(len(self.points) - 1):
            P0 = self.points[i]
            P1 = self.points[i + 1]

            # Fallback handles using compute_handles if needed
            if not P0[2] or not P1[3]:
                fallback_handles = self.compute_handles([P0, P1])
                handle_out = P0[2] if P0[2] else fallback_handles[0][0]
                handle_in = P1[3] if P1[3] else fallback_handles[1][1]
            else:
                handle_out = P0[2]
                handle_in = P1[3]

            p0 = (self.time_to_x(P0[0]), self.value_to_y(P0[1]))
            p3 = (self.time_to_x(P1[0]), self.value_to_y(P1[1]))
            p1 = (self.time_to_x(handle_out[0]), self.value_to_y(handle_out[1]))
            p2 = (self.time_to_x(handle_in[0]), self.value_to_y(handle_in[1]))

            prev_point = p0
            steps = 30
            for step in range(1, steps + 1):
                t = step / steps
                x, y = self.cubic_bezier(t, p0, p1, p2, p3)
                painter.drawLine(int(prev_point[0]), int(prev_point[1]), int(x), int(y))
                prev_point = (x, y)


        painter.setPen(QPen(Qt.blue, 2))

        for i in range(len(self.points) - 1):
            P0 = self.points[i]
            P1 = self.points[i + 1]

            # Use actual stored handles
            p0 = (self.time_to_x(P0[0]), self.value_to_y(P0[1]))
            p3 = (self.time_to_x(P1[0]), self.value_to_y(P1[1]))

            # Use stored handle_out from P0 and handle_in from P1
            handle_out = P0[2] if P0[2] else P0[:2]
            handle_in = P1[3] if P1[3] else P1[:2]

            p1 = (self.time_to_x(handle_out[0]), self.value_to_y(handle_out[1]))
            p2 = (self.time_to_x(handle_in[0]), self.value_to_y(handle_in[1]))

            prev_point = p0
            steps = 30
            for step in range(1, steps + 1):
                t = step / steps
                x, y = self.cubic_bezier(t, p0, p1, p2, p3)
                painter.drawLine(int(prev_point[0]), int(prev_point[1]), int(x), int(y))
                prev_point = (x, y)


    def draw_playhead(self, painter):
        if not self.timer.isActive() and self.playhead_time <= 0:
            return
        x = self.time_to_x(self.playhead_time)
        painter.setPen(QPen(Qt.green, 2))
        painter.drawLine(int(x), self.padding, int(x), self.height() - self.padding)

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
        val, ok = QInputDialog.getDouble(self, f"Set Y {which.upper()}", f"Enter new Y {which} value:", current_val, -10000, 10000, 2)
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
        val, ok = QInputDialog.getDouble(self, "Set X Max", "Enter new X max (seconds):", current_val, 0.1, 10000, 2)
        if ok:
            if val <= 0:
                QMessageBox.warning(self, "Invalid Value", "X max must be positive and greater than 0.")
                return
            self.duration = val
            self.points = [(min(t, self.duration), v) + tuple(pt[2:]) for pt in self.points]
            self.update()

    def mousePressEvent(self, event):
        x, y = event.x(), event.y()
        if event.button() == Qt.RightButton:
            if x < self.padding:
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
        t, v = self.x_to_time(x), self.y_to_value(y)
        if self.set_start_mode:
            self.playhead_time = max(0.0, min(self.duration, t))
            self.frame = int((self.playhead_time / self.duration) * self.fps * self.duration)
            self.set_start_mode = False
            self.update()
            return
        
        if self.add_mode:
            self.save_state()
            self.points.append((t, v, None, None))
            self.points.sort(key=lambda pt: pt[0])  # Ensure time order

            # Only compute handles if both in/out handles are None
            updated_points = []
            handles = self.compute_handles(self.points)
            for i, pt in enumerate(self.points):
                if pt[2] is None and pt[3] is None:
                    updated_points.append((pt[0], pt[1], handles[i][0], handles[i][1]))
                else:
                    updated_points.append(pt)
            self.points = updated_points

            self.update()

        elif event.button() == Qt.LeftButton:
            for i, pt in enumerate(self.points):
                px, py = self.time_to_x(pt[0]), self.value_to_y(pt[1])
                if abs(px - x) < 8 and abs(py - y) < 8:
                    self.selected_point = i
                    return
        elif event.button() == Qt.RightButton:
            for i, pt in enumerate(self.points):
                px, py = self.time_to_x(pt[0]), self.value_to_y(pt[1])
                if abs(px - x) < 8 and abs(py - y) < 8:
                    menu = QMenu(self)
                    action = menu.addAction("Delete Point")
                    action.triggered.connect(lambda _, idx=i: self.delete_point(idx))
                    menu.exec_(event.globalPos())
                    return
        
        # After detecting control points...
        for i, (t, v, handle_out, handle_in) in enumerate(self.points):
            cx, cy = self.time_to_x(t), self.value_to_y(v)
            if handle_out:
                hx, hy = self.time_to_x(handle_out[0]), self.value_to_y(handle_out[1])
                if abs(hx - x) < 8 and abs(hy - y) < 8:
                    self.selected_handle = (i, 'out')
                    return
            if handle_in:
                hx, hy = self.time_to_x(handle_in[0]), self.value_to_y(handle_in[1])
                if abs(hx - x) < 8 and abs(hy - y) < 8:
                    self.selected_handle = (i, 'in')
                    return


    def mouseMoveEvent(self, event):
        x, y = event.x(), event.y()
        t, v = self.x_to_time(x), self.y_to_value(y)
        self.hovered_point = None

        # Check hover over main control points
        for pt in self.points:
            px, py = self.time_to_x(pt[0]), self.value_to_y(pt[1])
            if abs(px - x) < 8 and abs(py - y) < 8:
                self.hovered_point = (pt[0], pt[1])
                break

        # Move main point
        if self.selected_point is not None:
            t = max(0.0, min(self.duration, t))
            v = max(self.value_min, min(self.value_max, v))
            h_out, h_in = self.points[self.selected_point][2:]
            self.points[self.selected_point] = (t, v, h_out, h_in)
            self.hovered_point = (t, v)

        # Move Bézier handle
        if self.selected_handle:
            i, direction = self.selected_handle
            t = max(0.0, min(self.duration, t))
            v = max(self.value_min, min(self.value_max, v))
            pt = self.points[i]
            h_out, h_in = pt[2], pt[3]
            if direction == 'out':
                h_out = (t, v)
            else:
                h_in = (t, v)
            self.points[i] = (pt[0], pt[1], h_out, h_in)

        self.update()


    def mouseReleaseEvent(self, event):
        self.selected_point = None
        self.selected_point = None
        self.selected_handle = None


    def delete_point(self, index):
        if 0 <= index < len(self.points):
            del self.points[index]
            self.update()

    def bezier_x(self, t, P0, P1, P2, P3):
        return (1 - t)**3 * P0[0] + 3 * (1 - t)**2 * t * P1[0] + 3 * (1 - t) * t**2 * P2[0] + t**3 * P3[0]

    def bezier_y(self, t, P0, P1, P2, P3):
        return (1 - t)**3 * P0[1] + 3 * (1 - t)**2 * t * P1[1] + 3 * (1 - t) * t**2 * P2[1] + t**3 * P3[1]

    def find_t_for_x(self, x_target, P0, P1, P2, P3, epsilon=1e-5):
        # Binary search for t in [0,1] so that bezier_x(t) ~ x_target
        low, high = 0.0, 1.0
        while low < high:
            mid = (low + high) / 2
            x_val = self.bezier_x(mid, P0, P1, P2, P3)
            if abs(x_val - x_target) < epsilon:
                return mid
            if x_val < x_target:
                low = mid
            else:
                high = mid
            if high - low < epsilon:
                return (low + high) / 2
        return 0.0

    def evaluate_spline_value_at_time(self, t, points):
        # Find segment containing t
        for i in range(len(points) - 1):
            if points[i][0] <= t <= points[i + 1][0]:
                P0 = (points[i][0], points[i][1])
                P1 = points[i][2]
                P2 = points[i + 1][3]
                P3 = (points[i + 1][0], points[i + 1][1])
                break
        else:
            # t out of range, clamp
            if t < points[0][0]:
                return points[0][1]
            else:
                return points[-1][1]

        local_t = self.find_t_for_x(t, P0, P1, P2, P3)
        return self.bezier_y(local_t, P0, P1, P2, P3)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SplineEditor()
    window.show()
    sys.exit(app.exec_())
