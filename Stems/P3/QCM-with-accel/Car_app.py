#region imports
from Car_GUI import Ui_Form
import sys
from PyQt5 import QtCore as qtc
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from QuarterCarModel import CarController
#endregion

class MainWindow(qtw.QWidget, Ui_Form):
    def __init__(self):
        """
        Main window constructor.
        """
        super().__init__()
        #call setupUi feom Ui_Form parent
        self.setupUi(self)

        #setup car controller
        input_widgets = (self.le_m1, self.le_v, self.le_k1, self.le_c1, self.le_m2, self.le_k2, self.le_ang, \
                         self.le_tmax, self.chk_IncludeAccel)
        display_widgets = (self.gv_Schematic, self.chk_LogX, self.chk_LogY, self.chk_LogAccel, \
        self.chk_ShowAccel, self.lbl_MaxMinInfo, self.layout_Plot)

        #instantiate the car controller
        self.controller = CarController((input_widgets, display_widgets))

        # connect signal to slots
        self.btn_calculate.clicked.connect(self.controller.calculate)
        self.pb_Optimize.clicked.connect(self.doOptimize)
        self.chk_LogX.stateChanged.connect(self.controller.doPlot)
        self.chk_LogY.stateChanged.connect(self.controller.doPlot)
        self.chk_LogAccel.stateChanged.connect(self.controller.doPlot)
        self.chk_ShowAccel.stateChanged.connect(self.controller.doPlot)
        self.controller.setupEventFilter(self)
        self.controller.setupCanvasMoveEvent(self)
        self.show()

    def eventFilter(self, obj, event):
        """
        This overrides the default eventFilter of the widget.  It takes action on events and then passes the event
        along to the parent widget.
        :param obj: The object on which the event happened
        :param event: The event itself
        :return: boolean from the parent widget
        """
        if obj == self.gv_Schematic.scene():
            et = event.type()
            if et == qtc.QEvent.GraphicsSceneMouseMove:
                scenePos = event.scenePos()
                strScene = "Mouse Position:  x = {}, y = {}".format(round(scenePos.x(), 2), round(-scenePos.y(), 2))
                self.setWindowTitle(strScene)  # display information in a label
            if event.type() == qtc.QEvent.GraphicsSceneWheel:  # I added this to zoom on mouse wheel scroll
                zm=self.controller.getZoom()
                if event.delta() > 0:
                    zm+=0.1
                else:
                    zm-=0.1
                zm=max(0.1,zm)
                self.controller.setZoom(zm)
        self.controller.updateSchematic()

        # pass the event along to the parent widget if there is one.
        return super(MainWindow, self).eventFilter(obj, event)

    def mouseMoveEvent_Canvas(self, event):
        if event.inaxes:
            self.controller.animate(event.xdata)
            ywheel,ybody, yroad, accel = self.controller.getPoints(event.xdata)
            self.setWindowTitle('t={:0.2f}(ms), y-road:{:0.3f}(mm), y-wheel:{:0.2f}(mm) , y-car:{:0.2f}(mm), accel={:0.2f}(g) '.format(event.xdata, yroad*1000, ywheel*1000, ybody*1000, accel))

    def doOptimize(self):
        app.setOverrideCursor(qtc.Qt.WaitCursor)
        self.controller.OptimizeSuspension()
        app.restoreOverrideCursor()

if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)
    mw = MainWindow()
    mw.setWindowTitle('Quarter Car Model')
    sys.exit(app.exec())
