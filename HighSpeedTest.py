import serial
import struct
import time
import csv
from queue import Queue
import threading

ComPort = 'COM3'
baudRate = 115200
output_file = "Muscle15_power_100g_4_8_25_50psi.csv"
BUFFER_SIZE = 1000

data_queue = Queue()
stop_event = threading.Event()

def serial_reader():
    with serial.Serial(ComPort, baudRate, timeout=1) as ser:
        while not stop_event.is_set():
            sync = ser.read(1)

            if sync and sync[0] == 0xAA:

                msg = ser.read(8)
                # print(list(msg))
                if len(msg) == 8:
                    try:
                        time_val, dist_val = struct.unpack('<ff', msg)
                        data_queue.put((time.time(), [time_val, dist_val]))
                    except struct.error:
                        continue

def data_writer():
    buffer = []
    dtBuffer = []
    last_time = time.time()

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Arduino Time (s)", "Distance"])  # header

        while not stop_event.is_set():
            try:
                t, data = data_queue.get(timeout=1)
                buffer.append(data)
                dtBuffer.append(t - last_time)
                last_time = t

                if len(buffer) >= BUFFER_SIZE:
                    writer.writerows(buffer)
                    meanFreq = 1 / (sum(dtBuffer) / len(dtBuffer))
                    print(f"Batch written. Avg Freq: {meanFreq:.1f} Hz")
                    buffer.clear()
                    dtBuffer.clear()
            except:
                continue

reader_thread = threading.Thread(target=serial_reader)
writer_thread = threading.Thread(target=data_writer)

reader_thread.start()
writer_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    stop_event.set()
    reader_thread.join()
    writer_thread.join()
    print("Stopped.")
