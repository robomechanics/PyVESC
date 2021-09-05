from pyvesc.protocol.interface import encode_request, encode, decode
from pyvesc.VESC.messages import *
import time
import serial

class VESC_CAN_COMM(object):
    def __init__(self, serial_port, baudrate=115200, timeout=0.05):
        """
        :param serial_port: Serial device to use for communication (i.e. "COM3" or "/dev/tty.usbmodem0")
        :param baudrate: baudrate for the serial communication. Shouldn't need to change this.
        :param timeout: timeout for the serial communication
        """

        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
	self.motors={}

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.serial_port.is_open:
            self.serial_port.flush()
            self.serial_port.close()

    def addMotor(self,name,**kwargs):
    	self.motors[name] = VESC(self.serial_port,**kwargs)
