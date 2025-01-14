import sys
import serial
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
import struct
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