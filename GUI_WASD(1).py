
import serial
import sys
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout,QLabel,QGridLayout
from PyQt5.QtCore import Qt
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

def send_ctrl(control, ser):  # 飞行数据【rol-滚转角, pit-俯仰角, yaw-偏航角, thr-油门】
    cmd_Head = 'AAAF501D01'  # 控制信息头 AA AF[HEAD] 50[REMOTER] 1D 01(data[1-29])
    flydata = control  # 飞行数据
    Trim = '0000000000000000'  # Trim信息(不校准)
    Mode = '03000000'  # 飞行控制模式

    def float_to_hex(data):  # float --> Hex 小端字节序
        return (struct.pack('<f', data)).hex()

    # 拼接指令字符串
    send_str = cmd_Head + float_to_hex(flydata[0]) + float_to_hex(flydata[1]) + float_to_hex(flydata[2]) + float_to_hex(flydata[3]) + Trim + Mode
    u_byte = bytes.fromhex(send_str)

    # 校验和
    checksum = sum(u_byte) % 256
    send_str += f'{checksum:02X}'  # 添加校验和
    ser.write(bytes.fromhex(send_str))  # 发送到串口

# GUI类
class DroneControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Drone Control GUI')
        self.setGeometry(300, 300, 300, 200)
#主布局
        layout = QVBoxLayout()
 # 添加说明标签
        self.label = QLabel("WASD 控制无人机方向,Q/E 控制偏航,R/F 控制油门", self)
        layout.addWidget(self.label)
        self.takeoffLandButton = QPushButton('Takeoff/Land', self)
        self.takeoffLandButton.clicked.connect(self.onTakeoffLand)
        layout.addWidget(self.takeoffLandButton)
# 添加 WASD 控制按钮布局
        control_layout = QGridLayout()
        
        self.wButton = QPushButton('W', self)
        self.wButton.clicked.connect(lambda: self.handbuttonPress(Qt.Key_W))
        control_layout.addWidget(self.wButton, 0, 1)

        self.aButton = QPushButton('A', self)
        self.aButton.clicked.connect(lambda: self.handbuttonPress(Qt.Key_A))
        control_layout.addWidget(self.aButton, 1, 0)

        self.sButton = QPushButton('S', self)
        self.sButton.clicked.connect(lambda: self.handbuttonPress(Qt.Key_S))
        control_layout.addWidget(self.sButton, 1, 1)

        self.dButton = QPushButton('D', self)
        self.dButton.clicked.connect(lambda: self.handbuttonPress(Qt.Key_D))
        control_layout.addWidget(self.dButton, 1, 2)

        layout.addLayout(control_layout)
        self.flip1Button = QPushButton('Flip Front', self)
        self.flip1Button.clicked.connect(lambda: self.onFlip(1))
        layout.addWidget(self.flip1Button)

        self.setLayout(layout)

        # 初始化控制数据：滚转、俯仰、偏航、油门
        self.control_data = [0.0, 0.0, 0.0, 50.0]  # 默认值，油门50%

    def onTakeoffLand(self):
        takeoff_and_land(self.ser)

    def onFlip(self, enum):
        flip_4D(self.ser, enum)

    def sendControl(self):
        """发送控制指令到无人机"""
        send_ctrl(self.control_data, self.ser)

    def handbuttonPress(self, key):
        """处理键盘按键事件"""
        step = 5.0  # 每次按键调整的步长
        if key() == Qt.Key_W:  # W - 增加俯仰角（向前飞）
            self.control_data[1] += step
        elif key() == Qt.Key_S:  # S - 减小俯仰角（向后飞）
            self.control_data[1] -= step
        elif key() == Qt.Key_A:  # A - 增加滚转角（向左飞）
            self.control_data[0] -= step
        elif key() == Qt.Key_D:  # D - 减小滚转角（向右飞）
            self.control_data[0] += step
        elif key() == Qt.Key_Q:  # Q - 增加偏航角（左旋转）
            self.control_data[2] -= step
        elif key() == Qt.Key_E:  # E - 减小偏航角（右旋转）
            self.control_data[2] += step
        elif key() == Qt.Key_R:  # R - 增加油门
            self.control_data[3] += step
        elif key() == Qt.Key_F:  # F - 减小油门
            self.control_data[3] -= step

        # 限制值范围
        self.control_data[0] = max(-45.0, min(45.0, self.control_data[0]))  # 滚转范围
        self.control_data[1] = max(-45.0, min(45.0, self.control_data[1]))  # 俯仰范围
        self.control_data[2] = max(-180.0, min(180.0, self.control_data[2]))  # 偏航范围
        self.control_data[3] = max(0.0, min(100.0, self.control_data[3]))  # 油门范围

        # 打印控制数据（调试用）
        print("Control Data:", self.control_data)

        # 发送控制指令
        self.sendControl()
def keyPressEvent(self, event):
        """处理键盘按键事件"""
        self.onKeyPress(event.key())

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = DroneControlGUI()
    ser = serial.Serial('COM5', 57600, timeout=0.5)
    ex.ser = ser
    ex.show()
    sys.exit(app.exec_())
