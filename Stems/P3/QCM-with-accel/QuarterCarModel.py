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

#these imports are necessary for drawing a matplot lib graph on my GUI
#no simple widget for this exists in QT Designer, so I have to add the widget in code.
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
#endregion

#region class definitions
#region specialized graphic items
class MassBlock(qtw.QGraphicsItem):
    def __init__(self, CenterX, CenterY, width=30, height=10, parent=None, pen=None, brush=None, name='CarBody', label=None, mass=10):
        super().__init__(parent)
        self.x = CenterX
        self.y = CenterY
        self.y0 = self.y
        self.pen = pen
        self.brush = brush
        self.width = width
        self.height = height
        self.top = self.y - self.height/2
        self.left = self.x - self.width/2
        self.rect = qtc.QRectF(self.left, self.top, self.width, self.height)
        self.name = name
        self.label = label
        self.mass = mass
        self.transformation = qtg.QTransform()
        stTT = self.name +"\nx={:0.3f}, y={:0.3f}\nmass = {:0.3f}".format(self.x, self.y, self.mass)
        self.setToolTip(stTT)

    def setMass(self, mass=None):
        if mass is not None:
            self.mass=mass
            stTT = self.name + "\nx={:0.3f}, y={:0.3f}\nmass = {:0.3f}".format(self.x, self.y, self.mass)
            self.setToolTip(stTT)

    def boundingRect(self):
        bounding_rect = self.transformation.mapRect(self.rect)
        return bounding_rect

    def paint(self, painter, option, widget=None):
        self.transformation.reset()
        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        if self.brush is not None:
            painter.setBrush(self.brush)
        self.top = -self.height/2
        self.left = -self.width/2
        self.rect=qtc.QRectF( self.left, self.top, self.width, self.height)
        painter.drawRect(self.rect)
        font=painter.font()
        font.setPointSize(6)
        painter.setFont(font)
        text="mass = {:0.1f} kg".format(self.mass)
        fm = qtg.QFontMetrics(painter.font())
        painter.drawText(qtc.QPointF(-fm.width(text)/2.0,fm.height()/2.0), text)
        if self.label is not None:
            font.setPointSize(12)
            painter.setFont(font)
            painter.drawText(qtc.QPointF((self.width/2.0)+10,0), self.label)
        self.transformation.translate(self.x, self.y)
        self.setTransform(self.transformation)
        self.transformation.reset()
        # brPen=qtg.QPen()
        # brPen.setWidth(0)
        # painter.setPen(brPen)
        # painter.setBrush(qtc.Qt.NoBrush)
        # painter.drawRect(self.boundingRect())

class Wheel(qtw.QGraphicsItem):
    def __init__(self, CenterX, radius=10, roady=10, parent=None, penTire=None, penMass=None, brushWheel=None, brushMass=None, name='Wheel', mass=10):
        super().__init__(parent)
        self.x = CenterX
        self.road_y = roady
        self.radius = radius
        self.y = self.road_y-self.radius
        self.y0 = self.y
        self.penTire = penTire
        self.brushWheel = brushWheel
        self.penMass = penMass
        self.brushMass = brushMass
        self.rect = qtc.QRectF(self.x - self.radius, self.y - self.radius, self.radius*2, self.radius*2)
        self.name = name
        self.mass = mass
        self.transformation = qtg.QTransform()
        stTT = self.name +"\nx={:0.3f}, y={:0.3f}\nmass = {:0.3f}".format(self.x, self.y, self.mass)
        self.setToolTip(stTT)
        self.massWidth=2*self.radius*0.85
        self.massHeight=self.radius/3
        self.massBlock = MassBlock(CenterX, self.y, width=self.massWidth, height=self.massHeight, pen=penMass, brush=brushMass, name="Wheel Mass", mass=mass)

    def setMass(self, mass=None):
        if mass is not None:
            self.mass=mass
            self.massBlock.setMass(mass)
            stTT = self.name + "\nx={:0.3f}, y={:0.3f}\nmass = {:0.3f}".format(self.x, self.y, self.mass)
            self.setToolTip(stTT)

    def boundingRect(self):
        bounding_rect = self.transformation.mapRect(self.rect)
        return bounding_rect
    def addToScene(self, scene):
        scene.addItem(self)
        scene.addItem(self.massBlock)

    def paint(self, painter, option, widget=None):
        height = 2*(self.road_y-self.y)
        width = 2*self.radius
        if self.penTire is not None:
            painter.setPen(self.penTire)  # Red color pen
        if self.brushWheel is not None:
            painter.setBrush(self.brushWheel)
        left = -width/2.0
        top = self.y-height/2
        self.rect=qtc.QRectF(left, top, width, height)
        painter.drawEllipse(self.rect)
        self.massBlock.y=self.y
        point = qtc.QPointF(self.radius*1.1, 0.0)
        painter.drawText(point, self.name)
class LinearSpring(qtw.QGraphicsItem):
    def __init__(self, ptSt, ptEn, coilsWidth=10, coilsLength=30, parent=None, pen=None, name='Spring', label=None, k=10, nCoils=6):
        """
        This is my class for a spring,
        :param ptSt: a QPointF point
        :param ptEn: a QPointF point
        :param coilsWidth: float for width of coils part
        :param coilsLength: float for free length of coils part
        :param parent:
        :param pen: for drawing the lines
        :param brush: not used for this one
        :param name: just a convenient name
        :param label: text to be displayed beside the spring
        :param k: the spring constant
        :param nCoils: number of coils to be drawn
        """
        super().__init__(parent)
        self.stPt = ptSt
        self.enPt = ptEn
        self.freeLength = self.getLength() # this assumes the spring to be free on initial definition
        self.DL = self.length-self.freeLength
        self.centerPt=(self.stPt+self.enPt)/2.0
        self.pen = pen
        self.coilsWidth = coilsWidth
        self.coilsLength = coilsLength
        self.top = - self.coilsLength / 2
        self.left = -self.coilsWidth / 2
        self.rect = qtc.QRectF(self.left, self.top, self.coilsWidth, self.coilsLength)
        self.name = name
        self.label = label
        self.k = k
        self.nCoils = nCoils
        self.transformation = qtg.QTransform()
        stTT = self.name +"\nx={:0.1f}, y={:0.1f}\nk = {:0.1f}".format(self.centerPt.x(), self.centerPt.y(), self.k)
        self.setToolTip(stTT)

    def setk(self, k=None):
        if k is not None:
            self.k=k
            stTT = self.name + "\nx={:0.3f}, y={:0.3f}\nk = {:0.3f}".format(self.stPt.x(), self.stPt.y(), self.k)
            self.setToolTip(stTT)

    def boundingRect(self):
        bounding_rect = self.transformation.mapRect(self.rect)
        return bounding_rect
    def getLength(self):
        p=self.enPt-self.stPt
        self.length = math.sqrt(p.x()**2+p.y()**2)
        return self.length

    def getDL(self):
        self.DL=self.length-self.freeLength
        return self.DL

    def getAngleDeg(self):
        p=self.enPt-self.stPt
        self.angleRad=math.atan2(p.y(), p.x())
        self.angleDeg=180.0/math.pi*self.angleRad
        return self.angleDeg

    def paint(self, painter, option, widget=None):
        """
        My method for drawing the spring is:
        Step 1:  compute the rectangle that will contain the coils section
            - call self.getLength()
            - call self.getDL() # note that self.DL < 0 for compression > 0 for tension
            - self.rect = qtc.QRectF(-(self.coilLength+self.DL)/2, -self.coilWidth/2, (self.coilLength+self.DL), self.coilWidth)
        Step 2: draw lines representing the coils
        Step 3: draw lines connecting (-self.length/2,0) to (self.rect.left,0) and (self.rect.right,0) to (self.length/2,0)
        Step 4: draw little circles at (-self.length/2,0) and (self.length/2,0)
        Step 5: translate by (self.length/2,0)
        Step 6: rotate to self.angleDeg
        Step 7: translate to stPt
        Step 8: decorate with text
        :param painter:
        :param option:
        :param widget:
        :return:
        """
        self.transformation.reset()

        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        #Step 1:
        self.getLength()
        self.getAngleDeg()
        self.getDL()
        ht = self.coilsWidth
        wd = self.coilsLength+self.DL
        top = -ht / 2
        left = -wd / 2
        right = wd / 2
        self.rect=qtc.QRectF(left, top, wd, ht)
        #painter.drawRect(self.rect)
        #Step 2:
        painter.drawLine(qtc.QPointF(left,0), qtc.QPointF(left, ht/2))
        dX=wd/(self.nCoils)
        for i in range(self.nCoils):
            painter.drawLine(qtc.QPointF(left + i * dX, ht / 2), qtc.QPointF(left + (i + 0.5) * dX, -ht / 2))
            painter.drawLine(qtc.QPointF(left + (i+0.5) * dX, -ht / 2), qtc.QPointF(left + (i + 1) * dX, ht / 2))
        painter.drawLine(qtc.QPointF(right, ht/2), qtc.QPointF(right,0))
        #Step 3:
        painter.drawLine(qtc.QPointF(-self.length/2,0),qtc.QPointF(left,0))
        painter.drawLine(qtc.QPointF(right,0),qtc.QPointF(self.length/2,0))
        #Step 4:
        nodeRad = 2
        stRec=qtc.QRectF(-self.length/2-nodeRad, -nodeRad, 2*nodeRad, 2* nodeRad)
        enRec=qtc.QRectF(self.length/2-nodeRad, -nodeRad, 2*nodeRad, 2* nodeRad)
        painter.drawEllipse(stRec)
        painter.drawEllipse(enRec)
        #Step 5:
        self.transformation.translate(self.stPt.x(), self.stPt.y())
        self.transformation.rotate(self.angleDeg)
        self.transformation.translate(self.length/2,0)
        self.setTransform(self.transformation)
        #Step 6:
        # self.transformation.reset()
        # font=painter.font()
        # font.setPointSize(6)
        # painter.setFont(font)
        # text="k = {:0.1f} N/m".format(self.k)
        # fm = qtg.QFontMetrics(painter.font())
        # centerPt = (self.stPt+self.enPt)/2,0
        # painter.drawText(qtc.QPointF(-fm.width(text)/2.0,fm.height()/2.0), text)
        # if self.label is not None:
        #     font.setPointSize(12)
        #     painter.setFont(font)
        #     painter.drawText(qtc.QPointF((self.coilsWidth / 2.0) + 10, 0), self.label)


        self.transformation.reset()
        # brPen=qtg.QPen()
        # brPen.setWidth(0)
        # painter.setPen(brPen)
        # painter.setBrush(qtc.Qt.NoBrush)
        # painter.drawRect(self.boundingRect())
class DashPot(qtw.QGraphicsItem):
    def __init__(self, ptSt, ptEn, dpWidth=10, dpLength=30, parent=None, pen=None, name='Dashpot', label=None, c=10):
        """
        This is my class for a dashpot,
        :param ptSt: a QPointF point
        :param ptEn: a QPointF point
        :param dpWidth: float for width of dashpot part
        :param dpLength: float for free length of dashpot part
        :param parent:
        :param pen: for drawing the lines
        :param brush: not used for this one
        :param name: just a convenient name
        :param label: text to be displayed beside the dashpot
        :param c: the dashpot coefficient
        """
        super().__init__(parent)
        self.stPt = ptSt
        self.enPt = ptEn
        self.freeLength = self.getLength() # this assumes the dashpot to be free on initial definition
        self.DL = self.length-self.freeLength
        self.centerPt=(self.stPt+self.enPt)/2.0
        self.pen = pen
        self.dpWidth = dpWidth
        self.dpLength = dpLength
        self.top = - self.dpLength / 2
        self.left = -self.dpWidth / 2
        self.rect = qtc.QRectF(self.left, self.top, self.dpWidth, self.dpLength)
        self.name = name
        self.label = label
        self.c = c
        self.transformation = qtg.QTransform()
        stTT = self.name +"\nx={:0.1f}, y={:0.1f}\nc = {:0.1f}".format(self.centerPt.x(), self.centerPt.y(), self.c)
        self.setToolTip(stTT)

    def setc(self, c=None):
        if c is not None:
            self.c=c
            stTT = self.name + "\nx={:0.3f}, y={:0.3f}\nc = {:0.3f}".format(self.stPt.x(), self.stPt.y(), self.c)
            self.setToolTip(stTT)

    def boundingRect(self):
        bounding_rect = self.transformation.mapRect(self.rect)
        return bounding_rect
    def getLength(self):
        p=self.enPt-self.stPt
        self.length = math.sqrt(p.x()**2+p.y()**2)
        return self.length

    def getDL(self):
        self.DL=self.length-self.freeLength
        return self.DL

    def getAngleDeg(self):
        p=self.enPt-self.stPt
        self.angleRad=math.atan2(p.y(), p.x())
        self.angleDeg=180.0/math.pi*self.angleRad
        return self.angleDeg

    def paint(self, painter, option, widget=None):
        """
        My method for drawing the spring is:
        Step 1:  compute the rectangle that will contain the dashpot section
            - call self.getLength()
            - call self.getDL() # note that self.DL < 0 for compression > 0 for tension
            - self.rect = qtc.QRectF(-(self.dpLength)/2, -self.dpWidth/2, (self.dpLength), self.dpWidth)
        Step 2: draw lines representing the dashpot
        Step 3: draw lines connecting (-self.length/2,0) to (self.rect.left,0) and (self.rect.right,0) to (self.length/2,0)
        Step 4: draw little circles at (-self.length/2,0) and (self.length/2,0)
        Step 5: translate by (self.length/2,0)
        Step 6: rotate to self.angleDeg
        Step 7: translate to stPt
        Step 8: decorate with text
        :param painter:
        :param option:
        :param widget:
        :return:
        """
        self.transformation.reset()

        if self.pen is not None:
            painter.setPen(self.pen)  # Red color pen
        #Step 1:
        self.getLength()
        self.getAngleDeg()
        self.getDL()
        ht = self.dpWidth
        wd = self.dpLength
        top = -ht / 2
        left = -wd / 2
        right = wd / 2
        self.rect=qtc.QRectF(left, top, wd, ht)
        #painter.drawRect(self.rect)
        #Step 2:
        painter.drawLine(qtc.QPointF(left,-ht/2), qtc.QPointF(left, ht/2))
        painter.drawLine(qtc.QPointF(left,-ht/2), qtc.QPointF(right, -ht/2))
        painter.drawLine(qtc.QPointF(left,ht/2), qtc.QPointF(right, ht/2))
        painter.drawLine(qtc.QPointF(self.DL, ht/2*0.95), qtc.QPointF(self.DL, -ht/2*0.95))
        #Step 3:
        painter.drawLine(qtc.QPointF(-self.length/2,0),qtc.QPointF(left,0))
        painter.drawLine(qtc.QPointF(self.DL,0),qtc.QPointF(self.length/2,0))
        #Step 4:
        nodeRad = 2
        stRec=qtc.QRectF(-self.length/2-nodeRad, -nodeRad, 2*nodeRad, 2* nodeRad)
        enRec=qtc.QRectF(self.length/2-nodeRad, -nodeRad, 2*nodeRad, 2* nodeRad)
        painter.drawEllipse(stRec)
        painter.drawEllipse(enRec)
        #Step 5:
        self.transformation.translate(self.stPt.x(), self.stPt.y())
        self.transformation.rotate(self.angleDeg)
        self.transformation.translate(self.length/2,0)
        self.setTransform(self.transformation)
        #Step 6:
        # self.transformation.reset()
        # font=painter.font()
        # font.setPointSize(6)
        # painter.setFont(font)
        # text="k = {:0.1f} N/m".format(self.k)
        # fm = qtg.QFontMetrics(painter.font())
        # centerPt = (self.stPt+self.enPt)/2,0
        # painter.drawText(qtc.QPointF(-fm.width(text)/2.0,fm.height()/2.0), text)
        # if self.label is not None:
        #     font.setPointSize(12)
        #     painter.setFont(font)
        #     painter.drawText(qtc.QPointF((self.dpWidth / 2.0) + 10, 0), self.label)


        self.transformation.reset()
        # brPen=qtg.QPen()
        # brPen.setWidth(0)
        # painter.setPen(brPen)
        # painter.setBrush(qtc.Qt.NoBrush)
        # painter.drawRect(self.boundingRect())
class Road(qtw.QGraphicsItem):
    def __init__(self, x,y, width=30, height=10, parent=None, pen=None, brush=None, name='Road', label=None):
        super().__init__(parent)
        self.x = x
        self.y = y
        self.x0 = x
        self.y0 = y
        self.pen = pen
        self.brush = brush
        self.width = width
        self.height = height
        self.top = self.y - self.height/2
        self.left = self.x - self.width/2
        self.rect = qtc.QRectF(self.left, self.top, self.width, self.height)
        self.name = name
        self.label = label
        self.transformation = qtg.QTransform()

    def boundingRect(self):
        bounding_rect = self.transformation.mapRect(self.rect)
        return bounding_rect

    def paint(self, painter, option, widget=None):
        if self.pen is not None:
            painter.setPen(self.pen)
        if self.brush is not None:
            painter.setBrush(self.brush)
        self.top = self.y
        self.left = self.x-self.width/2
        self.right = self.x+self.width/2

        painter.drawLine(qtc.QPointF(self.left,self.top), qtc.QPointF(self.right,self.top))
        self.rect=qtc.QRectF( self.left, self.top, self.width, self.height)
        penOutline = qtg.QPen(qtc.Qt.NoPen)
        painter.setPen(penOutline)
        painter.setBrush(self.brush)
        painter.drawRect(self.rect)
#endregion

#region MVC for quarter car model
class CarModel():
    """
    I re-wrote the quarter car model as an object oriented program
    and used the MVC pattern.  This is the quarter car model.  It just
    stores information about the car and results of the ode calculation.
    """
    def __init__(self):
        """
        self.results to hold results of odeint solution
        self.t time vector for odeint and for plotting
        self.tramp is time required to climb the ramp
        self.angrad is the ramp angle in radians
        self.ymag is the ramp height in m
        """
        self.results = []
        self.roadPosData = []
        self.wheelPosData = []
        self.bodyPosData = []
        self.bodyAccelData = []
        self.tmax = 3.0  # limit of timespan for simulation in seconds
        self.timeData = np.linspace(0, self.tmax, 200)
        self.tramp = 1.0  # time to traverse the ramp in seconds
        self.angrad = 0.1
        self.ymag = 6.0 / (12 * 3.3)  # ramp height in meters.  default is 0.1515 m
        self.yangdeg = 45.0  # ramp angle in degrees.  default is 45
        self.results = None
        self.m1 = 450  # mass of car body in kg
        self.m2 = 20  # mass of wheel in kg
        self.c1 = 4500  # damping coefficient in N*s/m
        self.k1 = 15000  # spring constant of suspension in N/m
        self.k2 = 90000  # spring constant of tire in N/m
        self.v = 120.0  # velocity of car in kph
        #self.mink1=(self.m1*9.81)/(16.0*25.4/1000.0)
        self.mink1=(self.m1*9.81)/(6.0*25.4/1000.0)
        self.maxk1=(self.m1*9.81)/(3.0*25.4/1000.0)
        self.mink2=((self.m1+self.m2)*9.81)/(1.5*25.4/1000.0)
        self.maxk2=((self.m1+self.m2)*9.81)/(0.75*25.4/1000.0)
        self.accelBodyData=None
        self.accelMax=0
        self.accelLim=1.5
        self.SSE=0.0

class CarView():
    def __init__(self, args):
        self.input_widgets, self.display_widgets = args
        # unpack widgets with same names as they have on the GUI
        self.le_m1, self.le_v, self.le_k1, self.le_c1, self.le_m2, self.le_k2, self.le_ang, \
         self.le_tmax, self.chk_IncludeAccel = self.input_widgets

        self.gv_Schematic, self.chk_LogX, self.chk_LogY, self.chk_LogAccel, \
        self.chk_ShowAccel, self.lbl_MaxMinInfo, self.layout_Plot = self.display_widgets

        # creating a canvas to draw a figure for the car model
        self.figure = Figure(tight_layout=True, frameon=True, facecolor='none')
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.layout_Plot.addWidget(NavigationToolbar2QT(self.canvas))
        self.layout_Plot.addWidget(self.canvas)

        # axes for the plotting using view
        self.ax = self.figure.add_subplot()
        if self.ax is not None:
            self.ax1 = self.ax.twinx()
        self.interpolatorWheel=None
        self.interpolatorBody=None
        self.interpolatorAccel=None
        self.posWheelTracer=None
        self.posBodyTracer=None
        self.posRoadTracer=None
        self.accelTracer=None
        self.buildScene()

    def updateView(self, model=None):
        self.le_m1.setText("{:0.2f}".format(model.m1))
        self.le_k1.setText("{:0.2f}".format(model.k1))
        self.le_c1.setText("{:0.2f}".format(model.c1))
        self.le_m2.setText("{:0.2f}".format(model.m2))
        self.le_k2.setText("{:0.2f}".format(model.k2))
        self.le_ang.setText("{:0.2f}".format(model.yangdeg))
        self.le_tmax.setText("{:0.2f}".format(model.tmax))
        stTmp="k1_min = {:0.2f}, k1_max = {:0.2f}\nk2_min = {:0.2f}, k2_max = {:0.2f}\n".format(model.mink1, model.maxk1, model.mink2, model.maxk2)
        stTmp+="SSE = {:0.2f}".format(model.SSE)
        self.lbl_MaxMinInfo.setText(stTmp)
        self.CarBody.setMass(model.m1)
        self.Wheel.setMass(model.m2)
        self.spring1.setk(model.k1)
        self.spring2.setk(model.k2)
        self.dashpot.setc(model.c1)
        self.doPlot(model)

    def buildScene(self):
        #create a scene object
        self.scene = qtw.QGraphicsScene()
        self.scene.setObjectName("MyScene")
        self.scene.setSceneRect(-200, -200, 400, 400)  # xLeft, yTop, Width, Height

        #set the scene for the graphics view object
        self.gv_Schematic.setScene(self.scene)
        #make some pens and brushes for my drawing
        self.setupPensAndBrushes()
        #make the components
        self.Road = Road(0,100,300,10,pen=self.groundPen,brush=self.groundBrush)
        self.Wheel = Wheel(0,50, roady=self.Road.y, penTire=self.penTire, brushWheel=self.brushWheel, penMass=self.penMass, brushMass=self.brushMass, name = "Wheel")
        self.CarBody = MassBlock(0, -70, 100, 30, pen=self.penMass, brush=self.brushMass, name="Car Body", label='Car Body', mass=150)
        self.spring1 = LinearSpring(qtc.QPointF(-35.0, self.Wheel.y), qtc.QPointF(-35, self.CarBody.y), pen=self.penMass, name='k1', label='Spring 1', k=10)
        self.spring2 = LinearSpring(qtc.QPointF(self.Wheel.x, self.Wheel.y), qtc.QPointF(self.Wheel.x, self.Road.y), coilsWidth=10, coilsLength=20, nCoils=4, pen=self.penMass, name='k2', label='Spring 2', k=10)
        self.dashpot = DashPot(qtc.QPointF(-self.spring1.stPt.x(), self.spring1.stPt.y()),qtc.QPointF(-self.spring1.enPt.x(), self.spring1.enPt.y()), dpWidth=10, dpLength=30, pen=self.penMass, name='c', label='Dashpot', c=10)
        self.scene.addItem(self.Road)
        self.Wheel.addToScene(self.scene)
        self.scene.addItem(self.CarBody)
        self.scene.addItem(self.spring1)
        self.scene.addItem(self.spring2)
        self.scene.addItem(self.dashpot)

    def setupCanvasMoveEvent(self, window):
        self.canvas.mpl_connect("motion_notify_event", window.mouseMoveEvent_Canvas)

    def setupEventFilter(self, window):
        self.gv_Schematic.setMouseTracking(True)
        self.gv_Schematic.scene().installEventFilter(window)
    def getZoom(self):
        return self.gv_Schematic.transform().m11()

    def setZoom(self, val):
        self.gv_Schematic.resetTransform()
        self.gv_Schematic.scale(val,val)
    def updateSchematic(self):
        self.scene.update()

    def setupPensAndBrushes(self):
        """
        Just sets up pens and brushes for drawing the schematic.
        :return:
        """
        self.penTire = qtg.QPen(qtg.QColor(qtc.Qt.black))
        self.penTire.setWidth(3)
        self.penMass = qtg.QPen(qtg.QColor("black"))
        self.penMass.setWidth(1)
        color = qtg.QColor(qtc.Qt.gray)
        color.setAlpha(64)
        self.brushWheel = qtg.QBrush(color)
        self.brushMass = qtg.QBrush(qtg.QColor(200,200,200, 64))
        self.groundPen = qtg.QPen(qtg.QColor(qtc.Qt.black))
        self.groundPen.setWidth(1)
        self.groundBrush = qtg.QBrush(qtg.QColor(qtc.Qt.black))
        self.groundBrush.setStyle(qtc.Qt.DiagCrossPattern)

    def doPlot(self, model=None):
        """
        Creates the plot.
        :param model:
        :return:
        """
        if model.results is None:
            return
        ax = self.ax
        ax1 = self.ax1
        # plot result of odeint solver
        QTPlotting = True  # assumes we are plotting onto a QT GUI form
        if ax == None:
            ax = plt.subplot()
            ax1 = ax.twinx()
            QTPlotting = False  # actually, we are just using CLI and showing the plot
        ax.clear()
        ax1.clear()
        t = model.timeData
        ywheel = model.wheelPosData
        ycar = model.bodyPosData
        accel = model.accelBodyData

        if self.chk_LogX.isChecked():
            ax.set_xlim(0.001,model.tmax)
            ax.set_xscale('log')
        else:
            ax.set_xlim(0.0, model.tmax)
            ax.set_xscale('linear')

        if self.chk_LogY.isChecked():
            ax.set_ylim(0.0001,max(ycar.max(), ywheel.max()*1.05))
            ax.set_yscale('log')
        else:
            ax.set_ylim(0.0, max(ycar.max(), ywheel.max()*1.05))
            ax.set_yscale('linear')

        ax.plot(t, ycar, 'b-', label='Body Position')
        ax.plot(t, ywheel, 'r-', label='Wheel Position')
        ax.plot(t, model.roadPosData, 'k-', label='Road Position', linewidth=3.0)
        if self.chk_ShowAccel.isChecked():
            ax1.plot(t, accel, 'g-', label='Body Accel')
            ax1.axhline(y=accel.max(), color='orange')  # horizontal line at accel.max()
            ax1.set_yscale('log' if self.chk_LogAccel.isChecked() else 'linear')
            ax1.set_ylabel("Y'' (g)", fontsize = 'large' if QTPlotting else 'medium')
            ax1.yaxis.set_label_coords(1.05,.5)

        # add axis labels
        ax.set_ylabel("Vertical Position (m)", fontsize='large' if QTPlotting else 'medium')
        ax.set_xlabel("time (s)", fontsize='large' if QTPlotting else 'medium')
        ax.legend()

        ax.axvline(x=model.tramp)  # vertical line at tramp
        ax.axhline(y=model.ymag)  # horizontal line at ymag
        # modify the tick marks
        ax.tick_params(axis='both', which='both', direction='in', top=True,
                       labelsize='large' if QTPlotting else 'medium')  # format tick marks
        ax1.tick_params(axis='both', which='both', direction='in', right=True,
                       labelsize='large' if QTPlotting else 'medium')  # format tick marks
        # show the plot
        if QTPlotting == False:
            plt.show()
        else:
            self.canvas.draw()

    def getPoints(self, model=None, t=0):
        """
        This gets interpolated values from the model data at a given time.
        :param model:
        :param t:
        :return: ywheel, ybody, yroad, accel
        """
        if model is None: return 0,0,0
        ywheel=np.interp(t, model.timeData, model.wheelPosData)
        ybody=np.interp(t, model.timeData, model.bodyPosData)
        yroad=np.interp(t,model.timeData, model.roadPosData)
        accel=np.interp(t, model.timeData, model.accelBodyData)
        return ywheel,ybody, yroad ,accel

    def animate(self, model=None, t=0):
        """
        Here I am interpolating the model data at time t and updating the vertical positions of the wheel and the car
        body.  The compression of the spring(s) occurs if I change their end point locations to something other
        than their free length which is established when they are first define.  Any change in the end point coordinates
        causes the active part of the spring to shorten/lengthen.  I could calculate the force of the spring by:
        F=k*deltaY.

        Similar idea for the dashpote.  Although in the case of the dashpot, it is the relative velocity difference
        between the end points that causes the force.
        :param model:
        :param t:
        :return:
        """
        if model is None: return
        ywheel, ybody, yroad, accel=self.getPoints(model, t)
        try:
            if self.posWheelTracer is not None:
                self.posWheelTracer.remove()
            if self.posBodyTracer is not None:
                self.posBodyTracer.remove()
            if self.posRoadTracer is not None:
                self.posRoadTracer.remove()
            if self.accelTracer is not None:
                self.accelTracer.remove()
        finally:
            pass
        # I'll assume a tire of size P275/55R20
        # This means the tire width is 275 mm -> 10.83 in
        # The height of the sidewall is:  0.55*10.85 = 5.95 in
        # The wheel diameter is 20 in
        # Therefore the tire diameter is 31.9 in
        # The Wheel radius is set to 100.  Hence scale is:  s*31.9(in)*[25.4(mm)/(in)]*[(m)/1000(mm)] = 200 => s=246.83/(m)
        self.scale  = 200*(1000/25.4)/self.Wheel.radius
        self.posWheelTracer=self.ax.plot(t, ywheel, 'o', markeredgecolor='red', markerfacecolor='none')[0]
        self.posBodyTracer=self.ax.plot(t, ybody, 'o', markeredgecolor='blue', markerfacecolor='none')[0]
        self.posRoadTracer=self.ax.plot(t, yroad, 'o', markeredgecolor='black', markerfacecolor='none')[0]
        self.Road.y=self.Road.y0-yroad*self.scale
        self.Wheel.road_y = self.Road.y
        self.Wheel.y=self.Wheel.y0-ywheel*self.scale
        self.spring2.enPt.setY(self.Road.y)
        self.spring2.stPt.setY(self.Wheel.y)
        self.CarBody.y=self.CarBody.y0-ybody*self.scale
        self.spring1.stPt.setY(self.Wheel.y)
        self.spring1.enPt.setY(self.CarBody.y)
        self.dashpot.stPt.setY(self.Wheel.y)
        self.dashpot.enPt.setY(self.CarBody.y)
        if self.chk_ShowAccel.isChecked():
            self.accelTracer=self.ax1.plot(t,accel,'o', markeredgecolor='green', markerfacecolor='none')[0]
        self.canvas.draw()
        self.scene.update()

class CarController():
    def __init__(self, args):
        """
        This is the controller I am using for the quarter car model.
        """
        self.input_widgets, self.display_widgets = args
        #unpack widgets with same names as they have on the GUI
        self.le_m1, self.le_v, self.le_k1, self.le_c1, self.le_m2, self.le_k2, self.le_ang, \
         self.le_tmax, self.chk_IncludeAccel = self.input_widgets

        self.gv_Schematic, self.chk_LogX, self.chk_LogY, self.chk_LogAccel, \
        self.chk_ShowAccel, self.lbl_MaxMinInfo, self.layout_horizontal_main = self.display_widgets

        self.model = CarModel()
        self.view = CarView(args)

    def ode_system(self, t ,X):
        # define the forcing function equation for the linear ramp
        # It takes self.tramp time to climb the ramp, so y position is
        # a linear function of time.
        if t < self.model.tramp:
            y = self.model.ymag * (t / self.model.tramp)
        else:
            y = self.model.ymag

        x1 = X[0]  # car position in vertical direction
        x1dot = X[1]  # car velocity  in vertical direction
        x2 = X[2]  # wheel position in vertical direction
        x2dot = X[3]  # wheel velocity in vertical direction

        # write the non-trivial equations in vertical direction
        x1ddot = (1 / self.model.m1) * (self.model.c1 * (x2dot - x1dot) + self.model.k1 * (x2 - x1))
        x2ddot = (1 / self.model.m2) * (
                    -self.model.c1 * (x2dot - x1dot) - self.model.k1 * (x2 - x1) + self.model.k2 * (y - x2))
        self.step += 1
        # return the derivatives of the input state vector
        return [x1dot, x1ddot, x2dot, x2ddot]

    def calculate(self, doCalc=True):
        """
        I will first set the basic properties of the car model and then calculate the result
        in another function doCalc.
        """
        #Step 1.  Read from the widgets
        self.model.m1 = float(self.le_m1.text())
        self.model.m2 = float(self.le_m2.text())
        self.model.c1 = float(self.le_c1.text())
        self.model.k1 = float(self.le_k1.text())
        self.model.k2 = float(self.le_k2.text())
        self.model.v = float(self.le_v.text())

        #recalculate min and max k values
        self.mink1=(self.model.m1*9.81)/(6.0*25.4/1000.0)
        self.maxk1=(self.model.m1*9.81)/(3.0*25.4/1000.0)
        self.mink2=((self.model.m1+self.model.m2)*9.81)/(1.5*25.4/1000.0)
        self.maxk2=((self.model.m1+self.model.m2)*9.81)/(0.75*25.4/1000.0)

        ymag=6.0/(12.0*3.3)   #This is the height of the ramp in m
        if ymag is not None:
            self.model.ymag = ymag
        self.model.yangdeg = float(self.le_ang.text())
        self.model.tmax = float(self.le_tmax.text())
        if(doCalc):
            self.doCalc()
        self.SSE((self.model.k1, self.model.c1, self.model.k2), optimizing=False)
        self.view.updateView(self.model)

    def setWidgets(self, w):
        """
        Pass widgets to view for setup.
        :param w:
        :return:
        """
        self.view.setWidgets(w)
        self.chk_IncludeAccel=self.view.chk_IncludeAccel

    def setupCanvasMoveEvent(self, window):
        """
        Pass through to view.
        :param window:
        :return:
        """
        self.view.setupCanvasMoveEvent(window)

    def setupEventFilter(self, window):
        self.view.setupEventFilter(window=window)
    def getZoom(self):
        """
        Pass request along to the view.
        :return:
        """
        return self.view.getZoom()

    def setZoom(self,val):
        """
        Pass request along to the view.
        :param val:
        :return:
        """
        self.view.setZoom(val=val)

    def updateSchematic(self):
        """
        Pass request along to the view.
        :return:
        """
        self.view.updateSchematic()

    def doCalc(self, doPlot=True, doAccel=True):
        """
        This solves the differential equations for the quarter car model.
        :param doPlot:
        :param doAccel:
        :return:
        """
        v = 1000 * self.model.v / 3600  # convert speed to m/s from kph
        self.model.angrad = self.model.yangdeg * math.pi / 180.0  # convert angle to radians
        self.model.tramp = self.model.ymag / (math.sin(self.model.angrad) * v)  # calculate time to traverse ramp

        self.model.timeData=np.logspace(np.log10(0.000001), np.log10(self.model.tmax), 2000)
        #self.model.timeData=np.linspace(0, self.model.tmax, 2000)
        self.model.roadPosData=[self.model.ymag if t>self.model.tramp else t*self.model.ymag/self.model.tramp for t in self.model.timeData]
        ic = [0, 0, 0, 0]
        # run odeint solver
        self.step=0
        self.model.results = solve_ivp(self.ode_system, t_span=[0,self.model.tmax], y0=ic, t_eval=self.model.timeData)
        if doAccel:
            self.calcAccel()
        if doPlot:
            self.doPlot()
        self.model.bodyPosData = self.model.results.y[0]
        self.model.wheelPosData = self.model.results.y[2]
        pass

    def calcAccel(self):
        """
        Calculate the acceleration in the vertical direction using the forward difference formula.
        """
        N=len(self.model.timeData)
        self.model.accelBodyData=np.zeros(shape=N)
        vel=self.model.results.y[1]
        for i in range(N):
            if i==N-1:
                h = self.model.timeData[i] - self.model.timeData[i - 1]
                self.model.accelBodyData[i]= (vel[i] - vel[i - 1]) / (9.81 * h)  # backward difference of velocity
            else:
                h = self.model.timeData[i + 1] - self.model.timeData[i]
                self.model.accelBodyData[i] = (vel[i + 1] - vel[i]) / (9.81 * h)  # forward difference of velocity
            # else:
            #     self.model.accel[i]=(vel[i+1]-vel[i-1])/(9.81*2.0*h)  # central difference of velocity
        self.model.accelMax=self.model.accelBodyData.max()
        return True

    def OptimizeSuspension(self):
        """
        Step 1:  set parameters based on GUI inputs by calling self.set(doCalc=False)
        Step 2:  make an initial guess for k1, c1, k2
        Step 3:  optimize the suspension
        :return:
        """
        #Step 1:
        #$JES MISSING CODE HERE$
        self.calculate(doCalc=False)
        #Step 2:
        #JES MISSING CODE HERE$
        x0=np.array([(self.model.mink1)*1.1, self.model.c1, (self.model.mink2)*1.1])
        #Step 3:
        #JES MISSING CODE HERE$
        answer=minimize(self.SSE,x0,method='Nelder-Mead')
        self.view.updateView(self.model)

    def SSE(self, vals, optimizing=True):
        """
        Calculates the sum of square errors between the contour of the road and the car body.
        :param vals:
        :param optimizing:
        :return:
        """
        k1, c1, k2=vals  #unpack the new values for k1, c1, k2
        self.model.k1=k1
        self.model.c1=c1
        self.model.k2=k2
        self.doCalc(doPlot=False)  #solve the odesystem with the new values of k1, c1, k2
        SSE=0
        for i in range(len(self.model.results.y[0])):
            t=self.model.timeData[i]
            y=self.model.results.y[0][i]
            if t < self.model.tramp:
                ytarget = self.model.ymag * (t / self.model.tramp)
            else:
                ytarget = self.model.ymag
            SSE+=(y-ytarget)**2

        #some penalty functions if the constants are too small
        if optimizing:
            if k1<self.model.mink1 or k1>self.model.maxk1:
                SSE+=100
            if c1<10:
                SSE+=100
            if k2<self.model.mink2 or k2>self.model.maxk2:
                SSE+=100
            o_IncludeAccel = self.chk_IncludeAccel.isChecked()
            # I'm overlaying a gradient in the acceleration limit that scales with distance from a target squared.
            if self.model.accelMax > self.model.accelLim and o_IncludeAccel:
                # need to soften suspension
                SSE+=10+10*(self.model.accelMax-self.model.accelLim)**2
        self.model.SSE=SSE
        return SSE

    def doPlot(self):
        self.view.doPlot(self.model)

    def animate(self, t):
        self.view.animate(self.model,t)

    def getPoints(self, t):
        return self.view.getPoints(self.model, t)
#endregion
#endregion

def main():
    QCM = CarController()
    QCM.doCalc()

if __name__ == '__main__':
    main()
