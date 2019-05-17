# Documentation for Every State Found in ./v_states.json
Here's how this works.

For each state, there's a TL;DR (meaning Too Long; Didn't Read) that tells the short definition of what each state does.

The technical definition of each value notes what each keypair in the JSON means within that particular instance.

### State 02
TL;DR: "Drive straight (in any of 4 directions) for a given distance (encoder count)"
Technical definition of each value:<br>
    - timeArray  : used in encoderDrive as value `speed`<br>
    - rightArray : used in encoderDrive as value `county`<br>
    - leftArray  : used in encoderDrive as value `countx`<br>
**Extra notes:** definition of encoderDrive (./smsAuton.java line 1051): public void encoderDrive(double countx, double county, double speed)<br>

### State 03
TL;DR: "Turn using the IMU"<br>
Technical definition of each value:<br>
    - timeArray  : //TODO<br>
    - rightArray : //TODO<br>
    - leftArray  : //TODO<br>

### State 04
TL;DR: "Lower the bot"<br>
Technical definition of each value: NO VALUES USED<br>

### State 05
TL;DR: "Drive using the TimeOfFlight sensor"<br>
Technical definition of each value:<br>
    - timeArray  : NOT USED<br>
    - rightArray :<br>
**Extra notes:** definition of onToF: boolean onToF(double leftspeed, double forwardspeed, double distance, double PCoeff)<br>

### State 06
LD;DR: "sensorAxis control"<br>
Technical definition of each value:<br>
    - timeArray  : NOT USED<br>
    - rightArray : Used to define position 2 (POS2) of the sensor<br>
    - leftArray  : Used to define position 1 (POS1) of the sensor<br>