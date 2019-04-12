package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="15555-simple-The-One-You-Should-Click-On-Because-It-Is-Totally-Not-Useless-get-stronger-metal", group="Pushbot")
public class smsSimple extends LinearOpMode {

    /* Declare OpMode members. */
    smsHardware robot = new smsHardware();   // Use a Pushbot's hardware

    float power = 0.05f;
    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello - This is smsSimple");    //
        telemetry.update();

        robot.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int minCollectorPosition = 0;
        int maxCollectorPosition = 1000;
        float POWERRDCR = 0.3f;
        int collectorCurrentPosition = robot.collector.getCurrentPosition();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Allow driver to select Tank vs POV by pressing START
            boolean dpad_check_up = gamepad1.dpad_up;
            boolean dpad_check_down = gamepad1.dpad_down;
            // gamepad2.left_stick_y;

            /*
            if (dpad_check_up && collectorCurrentPosition <= maxCollectorPosition) {
                robot.collector.setPower(power);
                telemetry.addData("Say", "The up D-pad is pressed");
            }
            if(dpad_check_down && collectorCurrentPosition >= minCollectorPosition) {
                robot.collector.setPower(-power);
                telemetry.addData("Say", "The down D-pad is pressed");
            }*/
            float DRIVE = gamepad2.left_stick_y * POWERRDCR;
            robot.collector.setPower(Range.clip(DRIVE, -1, 1));

            /*if ((dpad_check_down == false && dpad_check_up == false) || (collectorCurrentPosition <= minCollectorPosition || collectorCurrentPosition >= maxCollectorPosition)) {
                robot.collector.setPower(0);
            } else if (dpad_check_up && collectorCurrentPosition <= maxCollectorPosition) { // dpad is up
                robot.collector.setTargetPosition(maxCollectorPosition);
                telemetry.addData("[ROBOT STATE UPDATED]", String.format("dpad UP pressed. Moving to %d", maxCollectorPosition));
            }  else if (dpad_check_down && collectorCurrentPosition <= maxCollectorPosition) { // dpad is down
                robot.collector.setTargetPosition(minCollectorPosition);
                telemetry.addData("[ROBOT STATE UPDATED]", String.format("dpad DOWN pressed. Moving to %d", maxCollectorPosition));
            }*/

            collectorCurrentPosition = robot.collector.getCurrentPosition();
            telemetry.addLine(String.format("The minimum collector position is %d", minCollectorPosition));
            telemetry.addLine(String.format("The maximum collector position is %d", maxCollectorPosition));
            telemetry.addLine(String.format("The position of the collector motor is %d", collectorCurrentPosition));
            telemetry.update();
        }
    }
}