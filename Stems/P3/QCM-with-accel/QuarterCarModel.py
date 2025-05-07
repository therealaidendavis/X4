# -*- coding: utf-8 -*-

#region imports
from scipy.integrate import solve_ivp
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import math
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
#endregion

#region class definitions

# ... existing graphic item classes unchanged ...

#endregion

#region MVC for quarter car model
class CarModel():
    def __init__(self):
        self.results = []
        self.roadPosData = []
        self.wheelPosData = []
        self.bodyPosData = []
        self.bodyAccelData = []
        self.F_k1 = []
        self.F_k2 = []
        self.F_c1 = []
        self.tmax = 3.0
        self.timeData = np.linspace(0, self.tmax, 200)
        self.tramp = 1.0
        self.angrad = 0.1
        self.ymag = 6.0 / (12 * 3.3)
        self.yangdeg = 45.0
        self.results = None
        self.m1 = 450
        self.m2 = 20
        self.c1 = 4500
        self.k1 = 15000
        self.k2 = 90000
        self.v = 120.0
        self.mink1 = (self.m1*9.81)/(6.0*25.4/1000.0)
        self.maxk1 = (self.m1*9.81)/(3.0*25.4/1000.0)
        self.mink2 = ((self.m1+self.m2)*9.81)/(1.5*25.4/1000.0)
        self.maxk2 = ((self.m1+self.m2)*9.81)/(0.75*25.4/1000.0)
        self.accelBodyData = None
        self.accelMax = 0
        self.accelLim = 1.5
        self.SSE = 0.0

class CarView():
    def __init__(self, args):
        self.input_widgets, self.display_widgets = args
        self.le_m1, self.le_v, self.le_k1, self.le_c1, self.le_m2, self.le_k2, self.le_ang, self.le_tmax, self.chk_IncludeAccel = self.input_widgets
        self.gv_Schematic, self.chk_LogX, self.chk_LogY, self.chk_LogAccel, self.chk_ShowAccel, self.lbl_MaxMinInfo, self.layout_Plot, self.forcesLayout = self.display_widgets

        self.figure = Figure(tight_layout=True, frameon=True, facecolor='none')
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.layout_Plot.addWidget(NavigationToolbar2QT(self.canvas))
        self.layout_Plot.addWidget(self.canvas)

        self.figure2 = Figure(tight_layout=True, frameon=True, facecolor='none')
        self.canvas2 = FigureCanvasQTAgg(self.figure2)
        self.forcesLayout.addWidget(NavigationToolbar2QT(self.canvas2))
        self.forcesLayout.addWidget(self.canvas2)

        self.ax = self.figure.add_subplot()
        self.ax1 = self.ax.twinx() if self.ax else None
        self.ax_forces = self.figure2.add_subplot()
        self.buildScene()

    def doPlot(self, model=None):
        self.canvas.draw()
        self.plotForces(model)

    def plotForces(self, model):
        if not model.F_k1 or not model.F_k2 or not model.F_c1:
            return
        axf = self.ax_forces
        axf.clear()
        t = model.timeData
        axf.plot(t, model.F_k1, label="Spring Force (k1)")
        axf.plot(t, model.F_k2, label="Tire Force (k2)")
        axf.plot(t, model.F_c1, label="Dashpot Force (c1)")
        axf.set_xlabel("Time (s)")
        axf.set_ylabel("Force (N)")
        axf.set_title("Force vs. Time")
        axf.legend()
        axf.grid(True)
        self.canvas2.draw()

class CarController():
    def __init__(self, args):
        self.input_widgets, self.display_widgets = args
        self.model = CarModel()
        self.view = CarView(args)

    def calcAccel(self):
        N = len(self.model.timeData)
        self.model.accelBodyData = np.zeros(shape=N)
        vel = self.model.results.y[1]
        for i in range(N):
            h = self.model.timeData[i] - self.model.timeData[i - 1] if i > 0 else self.model.timeData[1] - self.model.timeData[0]
            if i == 0:
                self.model.accelBodyData[i] = (vel[i+1] - vel[i]) / (9.81 * h)
            elif i == N - 1:
                self.model.accelBodyData[i] = (vel[i] - vel[i-1]) / (9.81 * h)
            else:
                self.model.accelBodyData[i] = (vel[i+1] - vel[i-1]) / (9.81 * (2*h))
        self.model.accelMax = self.model.accelBodyData.max()
        return True

    def ode_system(self, t, X):
        if t < self.model.tramp:
            y = self.model.ymag * (t / self.model.tramp)
        else:
            y = self.model.ymag

        x1 = X[0]
        x1dot = X[1]
        x2 = X[2]
        x2dot = X[3]

        x1ddot = (1 / self.model.m1) * (self.model.c1 * (x2dot - x1dot) + self.model.k1 * (x2 - x1))
        x2ddot = (1 / self.model.m2) * (-self.model.c1 * (x2dot - x1dot) - self.model.k1 * (x2 - x1) + self.model.k2 * (y - x2))
        return [x1dot, x1ddot, x2dot, x2ddot]

    def doCalc(self, doPlot=True, doAccel=True):
        v = 1000 * self.model.v / 3600
        self.model.angrad = self.model.yangdeg * math.pi / 180.0
        self.model.tramp = self.model.ymag / (math.sin(self.model.angrad) * v)
        self.model.timeData = np.linspace(0, self.model.tmax, 2000)
        self.model.roadPosData = [self.model.ymag if t > self.model.tramp else t * self.model.ymag / self.model.tramp for t in self.model.timeData]
        ic = [0, 0, 0, 0]
        self.step = 0
        self.model.results = solve_ivp(self.ode_system, t_span=[0, self.model.tmax], y0=ic, t_eval=self.model.timeData)
        self.model.bodyPosData = self.model.results.y[0]
        self.model.wheelPosData = self.model.results.y[2]

        if doAccel:
            self.calcAccel()

        x1 = self.model.results.y[0]
        x1dot = self.model.results.y[1]
        x2 = self.model.results.y[2]
        x2dot = self.model.results.y[3]
        y = np.array(self.model.roadPosData)

        k1 = self.model.k1
        k2 = self.model.k2
        c1 = self.model.c1

        self.model.F_k1 = k1 * (x2 - x1)
        self.model.F_k2 = k2 * (y - x2)
        self.model.F_c1 = c1 * (x2dot - x1dot)

        if doPlot:
            self.doPlot()
#endregion
