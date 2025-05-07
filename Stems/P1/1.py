import sys
import numpy as np
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QHBoxLayout, QMessageBox
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ----- MODEL -----
class TakeoffModel:
    """Model class to compute takeoff distance (STO)."""
    def __init__(self):
        self.gc = 32.174  # lbm·ft/(lbf·s²)
        self.S = 0.5      # Simplified reference factor for lift

    def compute_sto(self, weight, thrust_array):
        """Returns STO array for given weight and thrust values."""
        return (weight ** 2) / (self.gc * self.S * thrust_array)


# ----- VIEW -----
class TakeoffView(QWidget):
    """View class that defines the GUI layout."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Takeoff Distance Calculator")

        # Input fields
        self.weight_label = QLabel("Weight (lb):")
        self.weight_input = QLineEdit()
        self.thrust_label = QLabel("Thrust (lbf):")
        self.thrust_input = QLineEdit()

        self.calc_button = QPushButton("Calculate")

        # Plot area
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        # Layouts
        input_layout = QHBoxLayout()
        input_layout.addWidget(self.weight_label)
        input_layout.addWidget(self.weight_input)
        input_layout.addWidget(self.thrust_label)
        input_layout.addWidget(self.thrust_input)
        input_layout.addWidget(self.calc_button)

        main_layout = QVBoxLayout()
        main_layout.addLayout(input_layout)
        main_layout.addWidget(self.canvas)

        self.setLayout(main_layout)


# ----- CONTROLLER -----
class TakeoffController:
    """Controller class connecting model and view."""
    def __init__(self, model, view):
        self.model = model
        self.view = view
        self.view.calc_button.clicked.connect(self.update_plot)

    def update_plot(self):
        """Handles button click and updates the graph."""
        try:
            weight = float(self.view.weight_input.text())
            thrust_val = float(self.view.thrust_input.text())
        except ValueError:
            QMessageBox.warning(self.view, "Input Error", "Please enter valid numbers.")
            return

        thrusts = np.linspace(10000, 80000, 300)
        weights = [weight - 10000, weight, weight + 10000]
        colors = ['blue', 'green', 'red']
        labels = ['W - 10,000', 'W', 'W + 10,000']

        ax = self.view.figure.clear()
        ax = self.view.figure.add_subplot(111)

        for w, color, label in zip(weights, colors, labels):
            stos = self.model.compute_sto(w, thrusts)
            ax.plot(thrusts, stos, label=label, color=color)

        # Add marker for user input thrust & weight
        sto_at_point = self.model.compute_sto(weight, np.array([thrust_val]))[0]
        ax.plot(thrust_val, sto_at_point, 'ko', label='Input Point')

        ax.set_xlabel("Thrust (lbf)")
        ax.set_ylabel("STO Distance (ft)")
        ax.set_title("Takeoff Distance vs Thrust")
        ax.legend()
        ax.grid(True)
        self.view.canvas.draw()


# ----- MAIN -----
def main():
    app = QApplication(sys.argv)
    model = TakeoffModel()
    view = TakeoffView()
    controller = TakeoffController(model, view)
    view.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
