# OttoDiesel_app.py

from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QApplication
import sys
from Air import air as Air
from Otto import ottoCycleView as Otto
from Diesel import dieselCycleView as Diesel
from Dual import DualCycle as Dual

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi("OttoDiesel_GUI.ui", self)

        self.btn_Calculate.clicked.connect(self.calculate_cycle)
        self.btn_Quit.clicked.connect(self.close)
        self.cmb_Cycle.addItems(["Otto", "Diesel", "Dual"])

    def calculate_cycle(self):
        cycle = self.cmb_Cycle.currentText()

        r = float(self.le_r.text())
        P1 = float(self.le_P1.text()) * 1e3  # kPa to Pa
        T1 = float(self.le_T1.text())

        if cycle == "Otto":
            obj = Otto(r, T1, P1)
        elif cycle == "Diesel":
            rc = float(self.le_rc.text())
            obj = Diesel(r, rc, T1, P1)
        elif cycle == "Dual":
            rc = float(self.le_rc.text())
            rp = float(self.le_rp.text())
            obj = Dual(r, rc, rp, T1, P1)
        else:
            return

        obj.solve()
        obj.plot()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
