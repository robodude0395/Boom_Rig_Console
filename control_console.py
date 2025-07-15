import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow
from spline_canvas import SplineCanvas
from PyQt5.QtCore import QObject, pyqtSignal

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("gui.ui", self)  # Load the UI file

        self.canvas = SplineCanvas(self.canvas_widget)

        # Connect buttons by their objectNames
        # Assuming you named buttons in the UI start_btn, stop_btn, reset_btn, add_btn
        self.play_button.clicked.connect(self.canvas.play)
        self.stop_button.clicked.connect(self.canvas.stop)
        self.restart_button.clicked.connect(self.canvas.reset)
        
        self.canvas.add_to_value_labels(self.value_label)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
