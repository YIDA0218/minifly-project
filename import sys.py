import sys
import serial
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import sys
from PyQt5.QtWidgets import QWidget, QCheckBox, QVBoxLayout, QGroupBox, QApplication, QHBoxLayout
from PyQt5 import QtCore
import struct
import scipy.signal as signal
from IIRFilter import IIRFilter
import time
# 串口通信函数
def takeoff_and_land(ser):
    cmd_onekey_fly = 'AAAF50020003AE'
    u_byte = bytes.fromhex(cmd_onekey_fly)
    ser.write(u_byte)

def fast_stop(ser):
    cmd_onekey_stop = 'AAAF50020004AF'
    u_byte = bytes.fromhex(cmd_onekey_stop)
    ser.write(u_byte)

def flip_4D(ser, enum):
    if enum == 1:
        cmd_onekey_flip = 'AAAF5003000501B2'
    elif enum == 2:
        cmd_onekey_flip = 'AAAF5003000502B3'
    elif enum == 3:
        cmd_onekey_flip = 'AAAF5003000503B4'
    elif enum == 4:
        cmd_onekey_flip = 'AAAF5003000504B5'
    u_byte = bytes.fromhex(cmd_onekey_flip)
    ser.write(u_byte)

def send_ctrl(control, ser):                # 飞行数据【rol-滚转角, pit-俯仰角, yaw-偏航角, thr-油门】
    cmd_Head = 'AAAF501D01'                 # 控制信息头 AA AF[HEAD] 50[REMOTER] 1D 01(data[1-29])
    
    flydata = control                       # 飞行数据【rol-滚转角, pit-俯仰角, yaw-偏航角, thr-油门】
    Trim = '0000000000000000'               # Trim信息(不校准)
    Mode = '03000000'                       # 飞行控制模式  # Mode = '00000000' 时为手动模式 # Mode = '01000000' 时为定高 # Mode = '03000000' 时为定点

    send_str = ''
    ########## send normal flight cmd
def float_to_hex(data):                 # float --> Hex 小端字节序
    return (struct.pack('<f', data)).hex()
    # calculate send bytes
    send_str = cmd_Head + float_to_hex(flydata[0]) + float_to_hex(flydata[1]) + float_to_hex(flydata[2]) + float_to_hex(flydata[3]) + Trim + Mode
    u_byte = bytes.fromhex(send_str)
    
    # checksum and add check sum to send bytes
    checksum = 0
    cnt = 0
    for a_byte in u_byte:
        checksum += a_byte
        cnt = cnt + 1
    H = hex(checksum % 256)
    if H[-2] == 'x':  # 0xF -> 0x0F
        send_str = send_str + '0' + H[-1]
    else:
        send_str = send_str + H[-2] + H[-1]
    ser.write(bytes.fromhex(send_str))   # 发送到串口（遥控器）实现飞 control cmd
##########

class DroneControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Drone Control GUI')
        self.setGeometry(300, 300, 300, 200)

        layout = QVBoxLayout()

        self.takeoffLandButton = QPushButton('Takeoff/Land', self)
        self.takeoffLandButton.clicked.connect(self.onTakeoffLand)
        layout.addWidget(self.takeoffLandButton)

        self.flip1Button = QPushButton('Flip Front', self)
        self.flip1Button.clicked.connect(lambda: self.onFlip(1))
        layout.addWidget(self.flip1Button)

        self.setLayout(layout)

    def onTakeoffLand(self):
        takeoff_and_land(self.ser)

    def onFlip(self, enum):
        flip_4D(self.ser, enum)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = DroneControlGUI()
    ser = serial.Serial('COM5', 57600, timeout=0.5)
    ex.ser = ser
    ex.show()
    sys.exit(app.exec_())
    dataset = np.loadtxt("datafor20s.txt")
i = 0

class PanelWindow(QWidget):
    def __init__(self):
        super(PanelWindow, self).__init__()
        self.state50 = False
        self.state26 = False

        # filtering box in Qt
        self.controlBox = QGroupBox("Filtering")  #创建一个画布
        self.checkBox1 = QCheckBox("50Hz Removal")  #放打钩按钮
        self.checkBox2 = QCheckBox("26Hz Removal")
        self.hbox_control = QHBoxLayout()  #在画布上放一个水平布局
        self.hbox_control.addWidget(self.checkBox1)  #在水平布局加东西
        self.hbox_control.addWidget(self.checkBox2)
        self.controlBox.setLayout(self.hbox_control)  #确定布局
        self.checkBox1.stateChanged.connect(self.check50)
        self.checkBox2.stateChanged.connect(self.check26)

        # # detection information box in Qt
        # self.info = "Welcome to action detection"
        # self.displayBox = QGroupBox("Detection")
        # self.label = QLabel(self.info)
        # self.vbox_display = QVBoxLayout()
        # self.vbox_display.addWidget(self.label)
        # self.displayBox.setLayout(self.vbox_display)

        # create a plot window
        self.fig1, self.ax1 = plt.subplots()  #创建绘图窗口1
        # print(self.ax1)
        # that's our plotbuffer
        self.plotbuffer = np.zeros(500)
        # create an empty line
        self.line, = self.ax1.plot(self.plotbuffer)  #横坐标
    # axis
        self.ax1.set_ylim(0, 1)    #设置y轴高度
        # That's our ringbuffer which accumluates the samples
        # It's emptied every time when the plot window below
        # does a repaint
        self.ringbuffer = []
        # add any initialisation code here (filters etc)

        # self.graphCanvas = FigureCanvas(plt.figure())
        # self.graphCanvas = MyMplCanvas(self)
        self.graphCanvas1 = FigureCanvas(self.fig1)
        # -------------------------------------------------
        self.fig2, self.ax2 = plt.subplots()  #绘图窗口2
        # print(self.ax2)
        # that's our plotbuffer
        self.plotbuffer = np.zeros(500)
        # create an empty line
        self.line, = self.ax2.plot(self.plotbuffer)
        # axis
        self.ax2.set_ylim(0, 1)  # 设置y轴高度
        # That's our ringbuffer which accumluates the samples
        # It's emptied every time when the plot window below
        # does a repaint
        self.ringbuffer = []
        # add any initialisation code here (filters etc)

        # self.graphCanvas = FigureCanvas(plt.figure())
        # self.graphCanvas = MyMplCanvas(self)
        self.graphCanvas2 = FigureCanvas(self.fig2)

        # put all widgets in vertical box layout
        vbox = QVBoxLayout()  #设置一个垂直布局
        vbox.addWidget(self.graphCanvas1)  #把图一放到垂直布局中
        vbox.addWidget(self.graphCanvas2)   #把图二放到垂直布局中
        vbox.addWidget(self.controlBox) #把水平布局放到垂直布局中
        self.setLayout(vbox)    #确认布局


        # start the animation
        self.ani1 = animation.FuncAnimation(self.fig1, self.update, interval=100)   #绘制动图
        self.ani2 = animation.FuncAnimation(self.fig2, self.update, interval=100)   #绘制动图

    def update(self, data):
        # add new data to the buffer
        self.plotbuffer = np.append(self.plotbuffer, self.ringbuffer)
        # only keep the 500 newest ones and discard the old ones
        self.plotbuffer = self.plotbuffer[-500:]
        self.ringbuffer = []
        # set the new 500 points of channel 9
        self.line.set_ydata(self.plotbuffer)
        return self.line,

    # appends data to the ringbuffer
    def addData(self, v):
        self.ringbuffer.append(v)

    # alter the state of 50Hz removing
    def check50(self, state):
        if (QtCore.Qt.Checked == state):
            self.state50 = True
        else:
            self.state50 = False

    # alter the state of 26Hz removing
    def check26(self, state):
        if (QtCore.Qt.Checked == state):
            self.state26 = True
        else:
            self.state26 = False
class IIRFilter:
    def __init__(self, sos):
        
        self.sos = sos
        self.zi = signal.sosfilt_zi(sos)  # 初始化滤波状态

    def filter(self, data):
        
        if isinstance(data, (list, np.ndarray)):  # 处理数组数据
            filtered_data, self.zi = signal.sosfilt(self.sos, data, zi=self.zi)
            return filtered_data
        else:  # 处理单个数据点
            filtered_data, self.zi = signal.sosfilt(self.sos, [data], zi=self.zi)
            return filtered_data[0]


samplingRate = 200
# init time
t0 = time.perf_counter()

det = []
sos = signal.butter(3*2, (45/samplingRate) * 2, 'lowpass', output='sos')
iirfilter = IIRFilter(sos)
remove26IirFilter = IIRFilter(signal.butter(3*2, [(25/samplingRate)*2, (27/samplingRate)*2], 'bandstop', output='sos'))
removeDCIIRFilter = IIRFilter(signal.butter(2*2, (0.02/samplingRate)*2, 'highpass', output='sos'))


def callBack(data):
    global i
    data = dataset[i]
    i = i + 1
    # send the sample to the plotwindow
    # add any filtering here:

    # realtimePlotWindow2.addData(data)
    # check the 50Hz removing state if it is true, filter the data
    # print(data)
    if (panelWindow.state50 == True):
        data = iirfilter.filter(data)
    # check the 26Hz removing state if it is true, filter the data
    if (panelWindow.state26 == True):
        data = remove26IirFilter.filter(data)
    # # detect the jump shooting.No detecting in the first 5 seconds because of the unstability of FIR filter initialization
    # if (isJumpShoting(data) > 1 and len(dataset) > 5 * samplingRate):
    #     # set the detection information in Qt
    #     shotingTime = len(dataset) / samplingRate
    #     panelWindow.label.setText("You are jump shoting at " + str(shotingTime) + "s")
    # check the sampling rate
    if ((time.perf_counter() - t0) % 1 < 0.01):
        print(len(dataset))

    panelWindow.addData(data)




# create the Qt window
app = QApplication(sys.argv)
panelWindow = PanelWindow()
panelWindow.show()



# show the plot and start the animation

app.exec()
# needs to be called to close the serial port


print("finished")
