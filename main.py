"""
HARP Continuum Robot Serial Communication Interface
By Eric Weissman
4/18/2025

Description:
This Python script provides a serial communication interface between a PC and a 
microcontroller (e.g., Arduino) for controlling a pneumatically actuated continuum robot 
(HARP). It sends target pressures to the robot and receives feedback on actual pressures 
via a custom binary protocol over a serial connection.

Functionality:
- Establishes a serial connection to the specified COM port at a specified baud rate.
- Continuously sends a data packet of 9 float values (target pressures) to the robot.
- Waits for a response packet from the robot containing 9 float values (measured pressures)
  and a single boolean indicating if the robot is ready for the next command.
- Uses threading to separate listening and writing operations for real-time performance.
- Maintains and prints message send/receive frequency to monitor communication health.

Data Protocol:
- Outgoing (from PC to Arduino):
    [0xAA][float x9][bool]
    - 0xAA: sync byte to signal start of message
    - 9 float values packed in little-endian format (target pressures)


- Incoming (from Arduino to PC):
    [0xAA][float x9]
    - 0xAA: sync byte
    - 9 float values (measured pressures)


Features:
- Uses Python's `threading` module to run listener and writer loops concurrently.
- Tracks and reports the frequency of communication (messages/sec) using timestamp buffers.
- Verbose logging can be toggled for debugging or performance monitoring.

Usage:
- Instantiate `SerialCommunicator` with the correct port, baudrate, and optional verbosity.
- Call `start()` to begin communication.
- Use `set_data_to_send()` to update the pressure command.
- Use `stop()` to gracefully end the communication loop.

Note:
Ensure the baudrate and COM port match the configuration on the Arduino.
"""

import serial
import struct
import threading
import time
from collections import deque
from queue import Queue


class SerialCommunicator:
    def __init__(self, port, baudrate, n_floats=9, verbose=True, direction = 'TwoWay'):

        self.directionOption = ['TwoWay', 'oneWayFromArduino' , 'OneWay2Arduino']
        if direction not in self.directionOption:
            raise Exception(f"Enter a valid direction from: {self.directionOption}")

        self.direction = direction
        self.port = port  # COM port for the Arduino
        self.baudrate = baudrate
        self.n_floats = n_floats
        self.sync_byte = 0xAA
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.running = False
        self.data_to_send = [0.0] * self.n_floats
        self.verbose = verbose
        self.expected_bytes = self.n_floats * 4
        self.pythonReady = True
        self.data_queue = Queue()

        # Deques to store recent timestamps for frequency calculation
        self.send_timestamps = deque(maxlen=100)
        self.recv_timestamps = deque(maxlen=100)
        self.send_counter = 0
        self.recv_counter = 0

    def start(self):
        time.sleep(1)
        self.running = True

        match self.direction:
            case 'TwoWay':
                threading.Thread(target=self.listen_thread_twoWay, daemon=True).start()
                threading.Thread(target=self.write_thread_twoWay, daemon=True).start()
            case 'oneWayFromArduino':
                threading.Thread(target=self.listen_thread_oneWayFromArduino, daemon=True).start()
            case 'OneWay2Arduino':
                threading.Thread(target=self.write_thread_OneWay2Arduino, daemon=True).start()

    def log_frequency(self, timestamps, label, counter_name):
        now = time.time()
        timestamps.append(now)
        counter = getattr(self, counter_name)
        counter += 1
        if len(timestamps) == timestamps.maxlen and counter >= timestamps.maxlen:
            duration = timestamps[-1] - timestamps[0]
            freq = len(timestamps) / duration if duration > 0 else 0
            print(f"[{label}] Avg Frequency (last {len(timestamps)}): {freq:.2f} messages/sec")
            counter = 0
        setattr(self, counter_name, counter)

    def listen_thread_twoWay(self):
        while self.running:
            if self.ser.in_waiting >= 1:  # if we have something to read
                # self.pythonReady = False
                sync = self.ser.read(1)  # read next byte
                if sync[0] == self.sync_byte:  # if the next byte is the sync bit
                    data_bytes = self.ser.read(self.expected_bytes)  # read the expected bytes

                    # process the data into floats
                    floats = [struct.unpack('<f', data_bytes[i * 4:i * 4 + 4])[0]
                              for i in range(self.n_floats)]

                    self.log_frequency(self.recv_timestamps, "Listener", "recv_counter")  # measure freq

                    if self.verbose:
                        print(f"[Listener] Received: {floats}")
            # self.pythonReady = True

    def write_thread_twoWay(self):
        while self.running:

            # lets buils our message as b'sync byte floats'
            payload = bytearray()
            payload.append(self.sync_byte)
            for value in self.data_to_send:
                payload.extend(struct.pack('<f', value))

            # payload.append(int(self.pythonReady))

            self.ser.write(payload)  # send message to the arduino

            self.log_frequency(self.send_timestamps, "Writer", "send_counter")  # measure freq

            if self.verbose:
                print(f"[Writer] Sent: {self.data_to_send}")

    def listen_thread_oneWayFromArduino(self):
        while self.running:

            sync = self.ser.read(1)

            if sync and sync[0] == self.sync_byte:
                data_bytes = self.ser.read(self.expected_bytes)
                if len(data_bytes) == self.expected_bytes:
                    try:
                        floats = [struct.unpack('<f', data_bytes[i * 4:i * 4 + 4])[0]
                                  for i in range(self.n_floats)]
                        self.data_queue.put(floats)

                        self.log_frequency(self.recv_timestamps, "Listener", "recv_counter")
                        if self.verbose:
                            print(f"[Listener] Received: {floats}")
                    except struct.error:
                        continue



    def write_thread_OneWay2Arduino(self):
        pass

    def stop(self):
        self.running = False
        self.ser.close()

    def set_data_to_send(self, new_data):
        if len(new_data) == self.n_floats:
            self.data_to_send = new_data
        else:
            raise ValueError(f"Expected {self.n_floats} floats")


if __name__ == "__main__":
    communicator = SerialCommunicator(port="COM3", baudrate=115200, n_floats=2, verbose=False, direction = 'oneWayFromArduino')
    communicator.start()

    try:
        while True:
            # Dynamically change data if needed
            communicator.set_data_to_send([1.1, 2.2])
            # communicator.set_listener_ready(True)  # or False

    except KeyboardInterrupt:
        communicator.stop()
        print("Communication stopped.")