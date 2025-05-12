import os
from serial_communicator import SerialCommunicator
import time

output_file= "Data/AnistropyEff_FR_test.csv"

if os.path.exists(output_file):
    raise FileExistsError(f"Error: The file '{output_file}' already exists. Choose a different name.")


communicator = SerialCommunicator(port="COM4",
                                  baudrate=115200,
                                  n_floats_to_arduino=1,
                                  n_floats_from_arduino=4,
                                  verbose=False,
                                  buffer_size=1000,
                                  direction = 'TwoWay',
                                  logFreq=True,
                                  CSVPath = output_file,
                                  DesiredFreq = 200)
communicator.start()
prevTime = time.time()
startTime = prevTime
freq = 100

while True:

    now = time.time()
    dt = now- prevTime
    prevTime = now

    if dt >= 1/freq:

        setP = [20.0]

        communicator.set_data_to_send(setP)
        buffer = communicator.get_buffer()
        if buffer:
            if buffer[-1][4]> 350 or buffer[-1][4]<50:
                 print("Error:check laser limits")
            print(buffer[-1][2])
    else:
        time.sleep(0.001)

