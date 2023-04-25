# File: main.py
import sys
import os
import platform
import re
from PySide2.QtUiTools import QUiLoader
from PySide2.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
from PySide2.QtWidgets import QApplication, QWidget
from PySide2.QtCore import QFile, QIODevice, QThread, QObject, Slot, Signal, QTimer

from matplotlib.backends.qt_compat import QtWidgets

import matplotlib

import folium
import io

import calendar
import time

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import numpy as np

import struct

import serial
from datetime import datetime 


window = []

serial_worker = []
image_worker = []

filename = None

class Connector(QObject):

    data_rx_sig = Signal(bytes)

    def __init__(self, port):
        QObject.__init__(self)
        self.serial = serial.Serial(port)
        self.serial.timeout = 0.2

    def send_data(self, data):
        self.serial.write(data)

    def receive_loop(self):
        while 1:
            data = self.serial.read_all()
            if(data):
                self.data_rx_sig.emit(data)
            else:
                break
        

    def start_rx(self, period):
        self.timer = QTimer()
        self.timer.timeout.connect(self.receive_loop)
        self.timer.start(period)





def rx_data_cb(data):
    print("receiving: ", data)
    f = open(filename, "a")
    f.write(data.decode("utf-8"))
    f.close()
    window.terminal.insertPlainText(data.decode("utf-8"))


image = b''

def rx_image_cb(data):
    global image
    print("got image fragment")
    image += data
    
def rx_image_complete_cb():
    global image
    date_time = create_filename()
    filename = "image_{}.jpeg".format(date_time)
    f = open(filename, "wb")
    f.write(image)
    f.close()


def send_data_trig():
    data = window.send_line.text()
    # window.send_line.clear()
    serial_worker.send_data(data.encode('utf-8'))
    print(data)


def create_filename():
    date_time = datetime.now().strftime("%Y%m%d%H%M%S")
    print("Current timestamp:", date_time)
    return date_time


if __name__ == "__main__":

    print(sys.argv)

    app = QApplication(sys.argv)

    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    ui_file_name = "mainwindow.ui"
    ui_file = QFile(ui_file_name)
    if not ui_file.open(QIODevice.ReadOnly):
        print("Cannot open {}: {}".format(ui_file_name, ui_file.errorString()))
        sys.exit(-1)
    loader = QUiLoader()
    window = loader.load(ui_file)
    ui_file.close()
    if not window:
        print(loader.errorString())
        sys.exit(-1)

    

    #CREATE LOG FILE
 
    
    date_time = create_filename()
    filename = "data_{}.log".format(date_time)
    f = open(filename, "w")
    f.write("Log file, date: {}\n".format(datetime.now()))
    f.close()
    


    #CREATE WORKER THREAD
    worker_thread = QThread()
    serial_worker = Connector(sys.argv[1])
    serial_worker.moveToThread(worker_thread)


    #CONNECT THREADED CALLBACKS
    serial_worker.data_rx_sig.connect(rx_data_cb)


    worker_thread.start()





    serial_worker.start_rx(0.5)


    window.send_btn.clicked.connect(send_data_trig)






    window.show()

    sys.exit(app.exec_())
