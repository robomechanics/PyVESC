from pyvesc.protocol.interface import encode_request, encode, decode
from pyvesc.VESC.messages import *
import time
import threading

class VESC(object):
    def __init__(self, serial_port, can_id=None, has_sensor=True, start_heartbeat=False):
        self.serial_port = serial_port
        self.can_id = can_id

        if has_sensor:
            self.serial_port.write(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF,can_id=self.can_id)))

        self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func)
        self._stop_heartbeat = threading.Event()

        if start_heartbeat:
            self.start_heartbeat()
        """
        # check firmware version and set GetValue fields to old values if pre version 3.xx
        version = self.get_firmware_version()
        if int(version.split('.')[0]) < 3:
            GetValues.fields = pre_v3_33_fields
        """

        # store message info for getting values so it doesn't need to calculate it every time
        msg = GetValues(can_id=self.can_id)
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._full_msg_size

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_heartbeat()

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.isSet():
            time.sleep(0.1)
            self.write(alive_msg)

    def start_heartbeat(self):
        """
        Starts a repetitive calling of the last set cmd to keep the motor alive.
        """
        self.heart_beat_thread.start()

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread and resets the last cmd function. THIS MUST BE CALLED BEFORE THE OBJECT GOES OUT OF
        SCOPE UNLESS WRAPPING IN A WITH STATEMENT (Assuming the heartbeat was started).
        """
        if self.heart_beat_thread.is_alive():
            self._stop_heartbeat.set()
            self.heart_beat_thread.join()

    def write(self, data, num_read_bytes=None):
        """
        A write wrapper function implemented like this to try and make it easier to incorporate other communication
        methods than UART in the future.
        :param data: the byte string to be sent
        :param num_read_bytes: number of bytes to read for decoding response
        :return: decoded response from buffer
        """
        try:
            self.serial_port.write(data)
            if num_read_bytes is not None:
                while self.serial_port.in_waiting <= num_read_bytes:
                    time.sleep(0.000001)  # add some delay just to help the CPU
                response, consumed = decode(self.serial_port.read(self.serial_port.in_waiting))
                return response
        except:
            return None

    def set_rpm(self, new_rpm):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        """
        self.write(encode(SetRPM(int(new_rpm*1000),can_id=self.can_id)))

    def set_current(self, new_current):
        """
        :param new_current: new current in milli-amps for the motor
        """
        self.write(encode(SetCurrent(new_current,can_id=self.can_id)))

    def set_duty_cycle(self, new_duty_cycle):
        """
        :param new_duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
        """
        self.write(encode(SetDutyCycle(new_duty_cycle,can_id=self.can_id)))

    def set_servo(self, new_servo_pos):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        """
        self.write(encode(SetServoPosition(new_servo_pos,can_id=self.can_id)))

    def get_measurements(self):
        """
        :return: A msg object with attributes containing the measurement values
        """
        return self.write(self._get_values_msg, num_read_bytes=self._get_values_msg_expected_length)

    def get_firmware_version(self):
        msg = GetVersion()
        return str(self.write(encode_request(msg,can_id=self.can_id), num_read_bytes=msg._full_msg_size))

    def get_rpm(self):
        """
        :return: Current motor rpm
        """
        return self.get_measurements().rpm

    def get_duty_cycle(self):
        """
        :return: Current applied duty-cycle
        """
        return self.get_measurements().duty_now

    def get_v_in(self):
        """
        :return: Current input voltage
        """
        return self.get_measurements().v_in

    def get_motor_current(self):
        """
        :return: Current motor current
        """
        return self.get_measurements().current_motor

    def get_incoming_current(self):
        """
        :return: Current incoming current
        """
        return self.get_measurements().current_in




