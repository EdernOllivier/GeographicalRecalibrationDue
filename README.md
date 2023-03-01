# GeographicalRecalibrationDue
There is a geographical recalibration to be executed by GPS and IMU with a simple Kalman filter on an UNO/Due Arduino board.

The GPS doesn't work properly at porting from the Arduino UNO to the Arduino Due.

Maybe that the patch to apply to hybride it has not succeeded.

Please do,

sudo chmod 666 /dev/ttyACM0

sudo tail -f /dev/ttyACM0

Then,

sudo apt install python3-pip

And,

pip install pyserial

Also,

python3 IMU2GPS.py

Then, press return to see the GPS data from the serial port of the Arduino USB UNO with Adafruit GPS board.
