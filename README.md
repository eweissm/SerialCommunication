# SerialCommunicator

`SerialCommunicator` is a Python class for robust, thread-based serial communication with an Arduino (or any serial device). It supports configurable float message exchange in multiple communication modes and includes a rolling buffer for real-time data access and analysis.

## Features

- ✅ Bidirectional communication over serial using `pyserial`
- ✅ Supports sending and receiving arrays of floats
- ✅ Configurable message size and communication direction
- ✅ Threaded send and receive loops (non-blocking)
- ✅ Synchronization byte for reliable message framing
- ✅ Optional verbose logging and frequency monitoring
- ✅ Constantly updating buffer with configurable size

## Installation

```bash
pip install pyserial
```


## Basic Usage (python side)

```
from serial_communicator import SerialCommunicator
import time

# Initialize
communicator = SerialCommunicator(
    port="COM3",               # Your Arduino's serial port
    baudrate=115200,
    n_floats_to_arduino=9,
    n_floats_from_arduino=9,
    direction="TwoWay",        # "TwoWay", "oneWayFromArduino", or "OneWay2Arduino"
    verbose=False,
    logFreq=True,
    buffer_size=50             # Number of recent messages to store
)

# Start communication
communicator.start()

# Continuously update outgoing data and monitor the buffer
try:
    while True:
        time.sleep(1)
        communicator.set_data_to_send([1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9])
        print("Buffer:", communicator.get_buffer())
except KeyboardInterrupt:
    communicator.stop()
    print("Communication stopped.")
```

## Basic Usage (Arduino side)

```
#define nPins 9  // number of pins
const int maxP = 35;  // saftey limit for pressure to be sent to the muscles
const uint8_t syncByte = 0xAA;

//store regulator pins
byte SetPointPins[nPins] = { 2, 3, 4, 5, 6, 7, 8, 9, 10 };
byte ActualPointPins[nPins] = { A7, A11, A3, A15, A2, A6, A10, A14, A1 };


int setPressure[nPins];
float ActualP[nPins];
bool readyToSend = false;
size_t expectedBytes = nPins * sizeof(float);

void setup() {

  //start serial coms
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial to initialize
  //set pinmode for all of the pin, initialize setPressure
  for (unsigned i = 0; i < nPins; i++) {
    pinMode(SetPointPins[i], OUTPUT);
    pinMode(ActualPointPins[i], INPUT);
    setPressure[i] = 0.0;
  }

}

void loop() {

  if (Serial.available() >= 1) { // if message in serial buffer
    uint8_t incoming = Serial.read(); // read next byte

    if (incoming == syncByte) { // if the byte is sync byte, we can read the rest of the message

      while(Serial.available() < expectedBytes ){} //wait to recieve the entire message

      for (size_t i = 0; i < nPins; ++i) {//store next 4*n bytes and decode them as floats

        byte floatBytes[4];
        for (int j = 0; j < 4; ++j) {
          floatBytes[j] = Serial.read();
        }
        float value;
        memcpy(&value, floatBytes, sizeof(float));
        setPressure[i] = min(maxP, int(value)); //convert to constrained int
      }


    }
  }

// set pressures and read actual pressures
  for (int i = 0; i < nPins; i++) {
    //set commanded Pressures
    int SetVoltage = map(setPressure[i], 0, 145, 0, 255);
    analogWrite(SetPointPins[i], SetVoltage);

    //read actual Pressures
    int PressureSensorVal = analogRead(ActualPointPins[i]);
    // ActualP[i] = float(PressureSensorVal) * (145. / 809.) - 38.3;
    ActualP[i] =setPressure[i];
  }
  

    //send our message 
    Serial.write(syncByte);
    for (int i = 0; i < nPins; i++) {
      Serial.write((uint8_t*)&ActualP[i], sizeof(float));

  }
  delayMicroseconds(10);

}
```


# Constructor Arguments

-port:	(str)	Serial port name (e.g., "COM3" or "/dev/ttyUSB0")

-baudrate:	(int)	Baud rate for serial communication

-n_floats_to_arduino:	(int)	Number of floats to send to Arduino

-n_floats_from_arduino:	(int)	Number of floats expected from Arduino

-direction:	(str)	"TwoWay", "oneWayFromArduino", or "OneWay2Arduino"

-verbose:	(bool)	Whether to print sent/received data

-logFreq:	(bool)	Whether to log message frequency

-buffer_size: (int)	Size of the rolling buffer for received messages

-CVSPath: (str) Optional input to record data recived from arduino to a csv. You can edit data_writer def to alter what is stored in the csv

# Methods
- start(): Begin communication threads based on the selected mode.

- stop(): Cleanly stop communication and close the serial port.

- set_data_to_send(data: list[float]): Update the float array to send.

- get_buffer(): Retrieve the list of the most recent received float arrays.

# Dependencies
- pyserial

# License
MIT License — free to use and modify.


# Known BUGS
- The freq control for two way comms is busted. Use an arbitrarily high freq to avoid issues (~5000 worked for me)
- 
