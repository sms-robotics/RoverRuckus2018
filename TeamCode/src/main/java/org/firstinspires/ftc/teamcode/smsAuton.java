package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


public class smsAuton extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AXKT92L/////AAAAGS+ZuWWofUShhb1MB4+Zbic/fnrONEJsEKNCY4RE1F7X8GaFg4EQYqHF4GMlj35ZJdzZ/LQlnXVV2WlhqhHR5IDlScqWtishwl2yPBRzCXAWYP5MCphLOigzPcshkggMYEKQWxwlhvoc2lsN+54KexfxlI0ss9cMq+unSD8ZZ5Of5OuY0lX7DWAEEPh1KsdeEU7EkCGP96f5TQI518LsriyHeg73KgDLCcGd0yBUSuGWTTV3o/cTRziN+Ac1sYNzw1sEddiBS2TfCdjRlY2qMmgyAMARQhYEbcqbzGz8jcDNOsX/gS/knjAZ9UYPZl7mYFyq3Acg3089CTN+EXkFEMJysFU0XQW9P2YzICsAivi5";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    smsHardware robot = new smsHardware();   // Use a Pushbot's hardware
    smsJSON settings = new smsJSON();
    ElapsedTime runtime = new ElapsedTime();

    static final double HEADING_THRESHOLD = 3;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.15;     // Larger is more responsive, but also less stable
    int CryptoBoxOffset = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

// Initialize the variables needed to store are JSON auton parameters
        int ArraySize = 50;
        double[] Arg1;
        Arg1 = new double[ArraySize];
        double[] Arg2;
        Arg2 = new double[ArraySize];
        double[] Arg3;
        Arg3 = new double[ArraySize];
        int[] v_state;
        v_state = new int[ArraySize];
        int v_state_current = 0;
        double servoPOS = 0.25;
        double servoPOS1 = 0.75;
        double servoPOS2 = 0.25;

        // Shared Code Below
        OpModeManagerImpl opModeManager = (OpModeManagerImpl) this.internalOpModeServices; //Store OpModeManagerImpl
        String OpModeName = robot.teamID + opModeManager.getActiveOpModeName() + ".json";
        if (robot.sensorAxis != null) robot.sensorAxis.setPosition(servoPOS);
        if (robot.servoMarker != null) robot.servoMarker.setPosition(1.0);
        settings.ReadSettings(OpModeName);
        v_state_current = 0;
        // Read the first step of the state machine
        v_state[v_state_current] = settings.GetIntSetting(String.format("v_state%02d", v_state_current));
        // Keep Reading until all steps are read, don't read anything we don't actually need
        while (v_state[v_state_current] > 0) {
            Arg1[v_state_current] = settings.GetSetting(String.format("Arg1_%02d", v_state_current));
            Arg2[v_state_current] = settings.GetSetting(String.format("Arg2_%02d", v_state_current));
            Arg3[v_state_current] = settings.GetSetting(String.format("Arg3_%02d", v_state_current));
            v_state_current++;
            v_state[v_state_current] = settings.GetIntSetting(String.format("v_state%02d", v_state_current));
        }
        telemetry.addData("name",OpModeName);
        telemetry.addData("Status", v_state_current);
        telemetry.update();
        v_state_current = 0;

        double intRunTime = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("v_state_current: ", v_state_current);
            telemetry.addData("v_state running: ", v_state[v_state_current]);

            switch (v_state[v_state_current]) {

                case 0:
                    break;

                case 1: // Drive straight (in any of 4 directions) for a given amount of time

                    robot.frontRightDrive.setPower(Arg3[v_state_current] + Arg2[v_state_current]);
                    robot.frontLeftDrive.setPower(Arg3[v_state_current] - Arg2[v_state_current]);
                    robot.rearRightDrive.setPower(Arg3[v_state_current] - Arg2[v_state_current]);
                    robot.rearLeftDrive.setPower(Arg3[v_state_current] + Arg2[v_state_current]);
                    intRunTime = runtime.milliseconds() + Arg1[v_state_current];
                    while (runtime.milliseconds() < intRunTime) {
                        telemetry.addData("drive-l", Arg3[v_state_current]);
                        telemetry.addData("drive-r", Arg2[v_state_current]);
                        telemetry.update();
                        idle();
                    }
                    robot.frontRightDrive.setPower(0.0);
                    robot.frontLeftDrive.setPower(0.0);
                    robot.rearRightDrive.setPower(0.0);
                    robot.rearLeftDrive.setPower(0.0);

                    v_state_current++;
                    break;

                case 2: // Drive straight (in any of 4 directions) for a given distance (encoder count)

                    encoderDrive(Arg3[v_state_current], Arg2[v_state_current], Arg1[v_state_current]);
                    v_state_current++;
                    break;

                case 3: // Turn using the IMU
                    while (opModeIsActive() && !onHeading(Arg3[v_state_current], Arg2[v_state_current], Arg1[v_state_current], P_TURN_COEFF)) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    v_state_current++;
                    break;

                case 4: // Lower the bot
                    // Start by raising the hook
                    robot.collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.collector.setTargetPosition(3600);
                    robot.collector.setPower(1.0f);
                    while (opModeIsActive() && robot.collector.isBusy()) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    // Now unlatch the hook
                    encoderDrive(0, 500, 0.3);
                    // Lower the hook
                    robot.collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.collector.setTargetPosition(0);
                    robot.collector.setPower(1.0f);

                    encoderDrive(-500, 0, 0.3);
                    //encoderDrive(200,0,0.3);
                    //encoderDrive(0,-200,0.3);

                    robot.collector.setPower(0.0f);
                    v_state_current++;
                    break;

                case 5: // Drive using the TimeOfFlight sensor
                    if (robot.sensorAxis != null) {
                        if ((Arg3[v_state_current] + Arg2[v_state_current]) * Arg2[v_state_current] == 0) {
                            servoPOS = servoPOS1;
                        } else {
                            servoPOS = servoPOS2;
                        }
                        if (Math.abs(robot.sensorAxis.getPosition() - servoPOS) > 0.05) {
                            robot.sensorAxis.setPosition(servoPOS);
                            if (robot.teamID == "15555") {
                                robot.collector.setTargetPosition(0);
                                robot.collector.setPower(1.0f);
                            }
                            sleep(500);
                            if (robot.teamID == "15555") robot.collector.setPower(0.0f);
                        }

                    }

                    while (opModeIsActive() && !onToF(Arg3[v_state_current], Arg2[v_state_current], Arg1[v_state_current], P_TURN_COEFF)) {
                        // Update telemetry & Allow time for other processes to run
                        telemetry.update();
                        idle();
                    }
                    v_state_current++;
                    break;

                case 6: // sensorAxis control
                    servoPOS1 = Arg3[v_state_current];
                    servoPOS2 = Arg2[v_state_current];
                    v_state_current++;
                    break;

                case 7: // push particle
                    NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
                    int color = colors.toColor();
                    float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                    colors.red /= max;
                    colors.green /= max;
                    colors.blue /= max;
                    color = colors.toColor();
                    if ((Color.red(color) * 100 / Color.blue(color)) > Arg1[v_state_current]) {
                        encoderDrive(0, 500, 0.2);
                        encoderDrive(0, -600, 0.2);
                        v_state_current += (int) (Arg3[v_state_current]);
                    }
                    v_state_current++;
                    break;

                case 8: // deliver team marker
                    robot.servoMarker.setPosition(0.5);
                    sleep(750);
                    v_state_current++;
                    break;

                case 12:
                    encoderDrive(3500, 0, 0.5);
                    robot.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armExtend.setTargetPosition(75);
                    robot.armExtend.setPower(0.3f);
                    if (robot.armMove != null) {
                        robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if (robot.teamID == "10645") {
                            robot.armMove.setTargetPosition(4500);
                        } else {
                            robot.armMove.setTargetPosition(5500);
                        }
                        robot.armMove.setPower(Math.abs(0.2));

                        while (opModeIsActive() && robot.armMove.isBusy()) {

                            float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                            robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                            robot.armExtend.setPower(0.2f);

                        }
                    }

                    robot.armExtend.setTargetPosition(robot.armExtend.getCurrentPosition() - 75);
                    robot.armExtend.setPower(0.2f);
                    if (robot.teamID == "10645") {
                        robot.collector.setPower(-1);
                    } else {
                        robot.collector.setPower(1);
                    }
                    sleep(2000);
                    robot.collector.setPower(0);

                    robot.armMove.setTargetPosition(0);
                    while (opModeIsActive() && robot.armMove.isBusy()) {

                        float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                        robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                        robot.armExtend.setPower(0.2f);

                    }

                    robot.armMove.setPower(Math.abs(0.3));


                    v_state_current++;
                    break;

                case 13:

                    robot.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armExtend.setTargetPosition(75);
                    robot.armExtend.setPower(0.3f);
                    if (robot.armMove != null) {
                        robot.armMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        {
                            robot.armMove.setTargetPosition(5500);
                        }
                        robot.armMove.setPower(Math.abs(0.2));

                        while (opModeIsActive() && robot.armMove.isBusy()) {

                            float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                            robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                            robot.armExtend.setPower(0.2f);

                        }
                    }

                    robot.armExtend.setTargetPosition(robot.armExtend.getCurrentPosition() - 75);
                    robot.armExtend.setPower(0.2f);
                {
                    robot.collector.setPower(-0.5);
                }
                sleep(2000);
                robot.collector.setPower(0);

                robot.armMove.setTargetPosition(0);
                robot.armMove.setPower(Math.abs(0.3));
                while (opModeIsActive() && robot.armMove.isBusy()) {

                    float aePos = (int) (Range.clip(robot.armMove.getCurrentPosition(), 0, 5500) / 7.11);
                    robot.armExtend.setTargetPosition((int) aePos); // based on a double 15:40 tooth reduction setup
                    robot.armExtend.setPower(0.2f);

                }

                v_state_current++;
                break;

                case 55:
                    /** Activate Tensor Flow Object Detection. */
                    if (tfod != null) {
                        tfod.activate();
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            //telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 3) {
                                int goldMineralX = -1;
                                int silverMineral1X = -1;
                                int silverMineral2X = -1;
                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                        Arg3[v_state_current] -= 25;
                                        Arg2[v_state_current] +=25;
                                        //telemetry.addData("Gold Mineral Position", "Left");
                                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                        //telemetry.addData("Gold Mineral Position", "Right");
                                    } else {
                                        Arg3[v_state_current] += 25;
                                        Arg2[v_state_current] -=25;
                                        //telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                }
                            }
                            //telemetry.update();
                        }
                        tfod.shutdown();

                    }
                    v_state_current++;
                    break;
                default:
                    v_state_current++;
                    break;
            }

            idle();
        }

    }

    // Not using rightspeed right now - only turning in place using leftspeed
    boolean onHeading(double leftspeed, double rightspeed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftspeed = 0.0;
            rightspeed = 0.0;
            onTarget = true;
        } else {
            steer = 1.0;
            if (Math.abs(error) <= HEADING_THRESHOLD * 5) { steer = 0.1 / (Math.abs(leftspeed) + Math.abs(rightspeed)); }
            leftspeed = leftspeed * steer;
            rightspeed = rightspeed * steer;
        }

        // Send desired speeds to motors.

        // Currently turning in place, not using rightspeed.

        robot.frontRightDrive.setPower(-leftspeed);
        robot.frontLeftDrive.setPower(+leftspeed);
        robot.rearRightDrive.setPower(-leftspeed);
        robot.rearLeftDrive.setPower(+leftspeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftspeed, rightspeed);

        return onTarget;
    }
    boolean onColor(double leftspeed, double rightspeed,double distance) {
        boolean onTarget = false;
        if (String.format(Locale.US, "%.02f", robot.colorRange.getDistance(DistanceUnit.CM))=="NaN") {
            onTarget = false;
        } else if (robot.colorRange.getDistance(DistanceUnit.CM) > distance) {
            onTarget = false;
        } else {
            onTarget = true;
        }
        if (!onTarget) { encoderDrive(leftspeed,rightspeed,0.1);}
        return onTarget;



    }
    boolean onToF(double leftspeed, double forwardspeed, double distance, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;

        // determine turn power based on +/- error
        error = getToFerror(distance);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftspeed = 0.0;
            forwardspeed = 0.0;
            onTarget = true;
        } else {
            steer = 1.0;
            if (Math.abs(error) <= HEADING_THRESHOLD * 5) { steer = 0.1 / (Math.abs(leftspeed) + Math.abs(forwardspeed)); }
            leftspeed = leftspeed * steer * -(error)/Math.abs((error));
            forwardspeed = forwardspeed * steer * -(error)/Math.abs((error));
        }

        // Send desired speeds to motors.
        robot.frontRightDrive.setPower(leftspeed + forwardspeed );
        robot.frontLeftDrive.setPower(leftspeed - forwardspeed);
        robot.rearRightDrive.setPower(leftspeed - forwardspeed);
        robot.rearLeftDrive.setPower(leftspeed + forwardspeed );

        // Display it for the driver
        telemetry.addData("Target", "%5.2f", distance);
        telemetry.addData("Err/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftspeed, forwardspeed);

        return onTarget;
    }


    /*
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (in degrees relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */

    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getToFerror(double targetDistance) {

        double robotError;
        robotError = targetDistance - robot.sensorRange.getDistance(DistanceUnit.CM);
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void encoderDrive(double countx, double county, double speed) {
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int FrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(countx + county);
        int FrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(countx - county);
        int RearRightTarget = robot.rearRightDrive.getCurrentPosition() + (int)(countx - county);
        int RearLeftTarget = robot.rearLeftDrive.getCurrentPosition() + (int)(countx + county);

        robot.frontRightDrive.setTargetPosition(FrontRightTarget);
        robot.frontLeftDrive.setTargetPosition(FrontLeftTarget);
        robot.rearRightDrive.setTargetPosition(RearRightTarget);
        robot.rearLeftDrive.setTargetPosition(RearLeftTarget);
        // start motion
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
        robot.rearRightDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        // keep looping while we are still active, and motors are running
        while (opModeIsActive() && (robot.frontRightDrive.isBusy() || robot.frontLeftDrive.isBusy() || robot.rearRightDrive.isBusy() || robot.rearLeftDrive.isBusy() )) {
            // Update telemetry & Allow time for other processes to run
            telemetry.update();
            idle();
        }
        // Stop all motion;
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);
        // Turn off RUN_TO_POSITION
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}

