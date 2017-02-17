#!/usr/bin/python3
# -*- coding: utf-8 -*-
#   Author: Andy Garcia
#   Date: 1/28/2017

import sys, os
from PyQt5.QtWidgets import (QWidget, QGridLayout, 
    QPushButton, QApplication, QCheckBox)
from PyQt5 import (QtGui,QtCore)
import serial
sys.path.append(os.path.dirname(__file__) + "../XboxController/")
from XboxController import XboxController

def contains(main_list,items):
    try:
        if type(items) is int:
            items = [items]
        if type(items) is QtCore.Qt.Key:
            items = [items]
        return all(x in main_list for x in items)
    except:
        print ("\nException")
        return False


class MainWindow(QWidget):
    
    def __init__(self):
        super().__init__()
        self.ser = None
        self.keys = []
        self.initUI()
        self.xbox_connected = False
        self.bluetooth_connected = False
        self.motor_powers = [50,50,50] #note that 0 is full reverse, 50 is stop, and 100 is full forward
        self.left_motor_power = 50  #note that 0 is full reverse, 50 is stop, and 100 is full forward
        self.right_motor_power = 50 #note that 0 is full reverse, 50 is stop, and 100 is full forward
        self.z_motor_power = 50     #note that 0 is full reverse, 50 is stop, and 100 is full forward


    def initUI(self):
        
        grid = QGridLayout()
        self.setLayout(grid)
    
        delay = 50
        interval = 10
        forward_button = QPushButton(text = "Forward")
        forward_button.setAutoRepeat(1)
        forward_button.setAutoRepeatDelay(delay)
        forward_button.setAutoRepeatInterval(interval)
        forward_button.pressed.connect(self.forward)

        reverse_button = QPushButton(text = "Reverse")
        reverse_button.setAutoRepeat(1)
        reverse_button.setAutoRepeatDelay(delay)
        reverse_button.setAutoRepeatInterval(interval)
        reverse_button.pressed.connect(self.reverse)

        right_button = QPushButton(text = "Right")
        right_button.setAutoRepeat(1)
        right_button.setAutoRepeatDelay(delay)
        right_button.setAutoRepeatInterval(interval)
        right_button.pressed.connect(self.right)

        left_button = QPushButton(text = "Left")
        left_button.setAutoRepeat(1)
        left_button.setAutoRepeatDelay(delay)
        left_button.setAutoRepeatInterval(interval)
        left_button.pressed.connect(self.left)

        sl_button = QPushButton(text = "Swing Left")
        sl_button.setAutoRepeat(1)
        sl_button.setAutoRepeatDelay(delay)
        sl_button.setAutoRepeatInterval(interval)
        sl_button.pressed.connect(self.swing_left)

        sr_button = QPushButton(text = "Swing Right")
        sr_button.setAutoRepeat(1)
        sr_button.setAutoRepeatDelay(delay)
        sr_button.setAutoRepeatInterval(interval)
        sr_button.pressed.connect(self.swing_right)

        bt_button = QPushButton(text = "Connect to Tank")
        bt_button.pressed.connect(self.bt_connect)

        bt_close_button = QPushButton(text = "Disconnect Tank")
        bt_close_button.pressed.connect(lambda: self.bt_close(self.ser))

        xbox_button = QCheckBox(text = "Use Xbox Controller")
        xbox_button.setChecked(False)
        xbox_button.stateChanged.connect(lambda:self.setXboxSend(xbox_button))

        grid.addWidget(forward_button,1,3)
        grid.addWidget(right_button,2,4)
        grid.addWidget(left_button,2,2)
        grid.addWidget(reverse_button,3,3)
        grid.addWidget(sl_button,1,2)
        grid.addWidget(sr_button,1,4)
        grid.addWidget(bt_button,1,1)
        grid.addWidget(bt_close_button,3,1)
        grid.addWidget(xbox_button,4,1)

        
        self.move(300, 150)
        self.setWindowTitle('AquaDrone Controller')
        self.setWindowIcon(QtGui.QIcon('tank.png'))
        self.show()


         


    def printMotorPower(self):
        if (self.left_motor_power < 50 and self.right_motor_power < 50):
            print ("Reverse at {0}%".format(self.left_motor_power*2))
        elif (self.left_motor_power >50 and self.right_motor_power > 50):
            print ("Forward at {0}%".format(self.left_motor_power*2))
        else:
            print ("Stop")

   

    def bt_connect(self,port = '/dev/cu.HC-05-DevB'):
        try:
            self.ser = serial.Serial(port)
            self.bluetooth_connected = True
            print ("Connected on " + self.ser.name)
        except ValueError:
            self.ser = None
            print ("Could not connect to " + port + ": Value Error")

        except serial.SerialException:
            self.ser = None
            print ("Could not connect to " + port + ": Device not found")
        
        except:
            self.ser = None
            print ("Could not connect to " + port + ":Unknown error")

    def bt_close(self,port):
        try:
            if (port.is_open):
                print ("Closing " + port.name)
                port.close()
                self.bluetooth_connected = False
            else:
                print (port.name + " is not open")
        except:
            print ("Invalid Port")

    def swing_right(self):
        try:
            print("Swing Right")
            self.ser.write(bytes([100,50,0,126]))
        except:
            print ("Could not send swing right command")

    def swing_left(self):
        try:
            print("Swing Left")
            self.ser.write(bytes([50,100,0,126]))
        except:
            print ("Could not send swing left command")

    def forward(self):
        try:
            print ("Forward")
            self.ser.write(bytes([100,100,0,126]))
        except:
            print ("Could not send forward command")

    def reverse(self):
        try:
            print ("Reverse")
            self.ser.write(bytes([0,0,0,126]))
        except:
            print ("Could not send reverse command")

    def right(self):
        try:
            print ("Right")
            self.ser.write(bytes([100,0,0,126]))
        except:
            print ("Could not send right command")

    def left(self):
        try:
            print ("Left")
            self.ser.write(bytes([0,100,0,126]))
        except:
            print ("Could not send left command")

    def keyPressEvent(self,event):
        if type(event) == QtGui.QKeyEvent:
            if (not contains(self.keys,event.key())):
                self.keys.append(event.key())
            self.multikey()
            event.accept()

    def keyReleaseEvent(self,event):
        if type(event) == QtGui.QKeyEvent:
            try:
                self.keys.remove(event.key())
            except:
                pass
    def setXboxSend(self,button):
        if button.isChecked() == True:
            try:
                #generic call back
                def controlCallBack(xboxControlId, value):
                    if (xboxControlId == 1):
                        self.left_y = value
                        #print ("Y axis: {0}".format(value))

                #setup xbox controller, set out the deadzone and scale, also invert the Y Axis (for some reason in Pygame negative is up - wierd! 
                self.xboxCont = XboxController(controlCallBack, deadzone = 10, scale = 50, invertYAxis = True)
                self.xboxCont.start()
                print ("Xbox 360 Controller Connected")

            except Exception as e:
                print (e)
                button.setChecked(False)
        else:
            try:
                self.xboxCont.stop()
                self.xboxCont = None
                print ("Xbox 360 Controller Disconnected")
            except:
                pass

    def multikey(self):
        if (contains(self.keys,[QtCore.Qt.Key_W,QtCore.Qt.Key_A])):
            self.swing_left()
        elif (contains(self.keys,[QtCore.Qt.Key_W,QtCore.Qt.Key_D])):
            self.swing_right()
        elif (contains(self.keys,QtCore.Qt.Key_W)):
            self.forward()
        elif (contains(self.keys,QtCore.Qt.Key_S)):
            self.reverse()
        elif (contains(self.keys,QtCore.Qt.Key_A)):
            self.left()
        elif (contains(self.keys,QtCore.Qt.Key_D)):
            self.right()
        elif (contains(self.keys,QtCore.Qt.Key_Space)):
            try:
                send = []
                #send.append(int(self.xboxCont.left_y))
                send.append(int(self.xboxCont.LTHUMBY))
                send.append(int(self.xboxCont.LTHUMBY))
                send.append(int(self.xboxCont.LTHUMBY))
                send.append(126)
                self.ser.write(bytes(send))
                print ("Sent {0}".format(send))
            except Exception as e:
                print ("Could not send xbox command")
                print (type(e))
                print(e.args)
                print (e)
        else:
            print ("Unknown Key: " + str(self.keys))

    def closeEvent(self,event):
        try:
            self.xboxCont.stop()
        except:
            pass
        event.accept()

if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    win = MainWindow()
    sys.exit(app.exec_())