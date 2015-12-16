package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.Servo;

/**
 * Autonomous mode
 * Created by Aryoman on 12/2/2015.
 * revised by Athena on 12/7/2015 - rotate method
 * revised vy Alex on 12/8/2015 - going forward set distance, rotate, go until touching something
 */
public class AutonomousOpMode extends LinearOpMode {


    final static int STOP_AT_DISTANCE_READING = 50;                     //ODS reading; robot will stop at this distance from wall
    final static int MID_POINT_ALPHA = 20;                              //(0 + 40) / 2 = 20
    final static double CORRECTION_MULTIPLIER = 0.012;                  //kp constant power
    final static double LINE_FOLLOW_MOTOR_POWER = -0.2;                 //power of the motor

    final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;
    //one wheel rotation covers ~12 inches
    //one tile is ~24 inches long
    //final static int TARGET_POSITION1 = 2 * ENCODER_COUNTS_PER_ROTATION;
    final static double DRIVING_POWER = 0.8;

    MecanumWheels mecanumWheels;

    /*
    Arm arm;
    Brushes brushes;
    Servo beacon6;
    */

    OpticalDistanceSensor opticalDistSensorRight;
    OpticalDistanceSensor opticalDistSensorLeft;
    ColorSensor colorSensorDrive;
    ColorSensor colorSensorBeacon;
    TouchSensor touchSensor;

    // gyro heading is from 0 to 359
    public void rotateToHeading(double requiredHeading) throws InterruptedException {

        mecanumWheels.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


        double MIN_ROTATE_POWER = 0.3;

        // make sure requiredHeading is positive and less than 360
        while (requiredHeading >= 360) {
            requiredHeading -= 360;
        }
        while (requiredHeading < 0) {
            requiredHeading += 360;
        }

        double power;
        // choose rotation direction
        double currentHeading = mecanumWheels.getGyroHeading();
        if (requiredHeading > currentHeading) {
            if (requiredHeading - currentHeading < 180) {
                power = MIN_ROTATE_POWER;
            } else {
                power = -MIN_ROTATE_POWER;
            }
        } else {
            if (requiredHeading + 360 - currentHeading < 180) {
                power = MIN_ROTATE_POWER;
            } else {
                power = -MIN_ROTATE_POWER;
            }
        }

        // start rotation fast, then slow down as you approach the required heading
        rotateToTolerance(60, requiredHeading, power * 2);
        rotateToTolerance(30, requiredHeading, power * 1.5);
        rotateToTolerance(1, requiredHeading, power);
        mecanumWheels.powerMotors(0, 0, 0);
        waitOneFullHardwareCycle();
    }

    // we want the delta to be less than 180 degrees
    // for example, the delta between 350 and 10 should be 20
    private double getHeadingDelta(double requiredHeading) {
        double headingDelta = Math.abs(requiredHeading - mecanumWheels.getGyroHeading());
        if (headingDelta > 180) {
            headingDelta = 360 - headingDelta;
        }
        return headingDelta;
    }

    // remember to zero the power after this method call if you want to stop rotation
    private void rotateToTolerance(double tolerance, double requiredHeading, double power) throws InterruptedException {
        double headingDelta = getHeadingDelta(requiredHeading);

        if (headingDelta > tolerance) {
            // power motors
            mecanumWheels.powerMotors(0, 0, power);
            waitOneFullHardwareCycle();
            // if current gyro heading is not close enough to the required heading
            // wait and check again
            while (headingDelta > tolerance && opModeIsActive()) {
                waitOneFullHardwareCycle();
                headingDelta = getHeadingDelta(requiredHeading);
                telemetry.addData("delta", headingDelta);
            }
        }
    }

    public boolean checkTouchObject() {
        telemetry.addData("LEFT touch", opticalDistSensorLeft.getLightDetectedRaw());
        telemetry.addData("RIGHT touch", opticalDistSensorRight.getLightDetectedRaw());
        return touchSensor.isPressed() || opticalDistSensorLeft.getLightDetectedRaw() > 10 || opticalDistSensorRight.getLightDetectedRaw() > 20;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true);
        mecanumWheels.backwardSetup();
        waitOneFullHardwareCycle();
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        colorSensorBeacon.setI2cAddress(0x3e);
        opticalDistSensorRight = hardwareMap.opticalDistanceSensor.get("Optical Distance Right");
        opticalDistSensorLeft = hardwareMap.opticalDistanceSensor.get("Optical Distance Left");
        touchSensor = hardwareMap.touchSensor.get("Touch Sensor");

        waitForStart();

        //to be able to use field coordinates in the following driving mode
        // our robot must be faced forward in the beginning

        // assuming we are starting on a tile next to  the mountain
        // go forward (back) 36 inches, which is  ~3 wheel rotation
        // OR
        // assuming we are on the second tile from the mountain
        // go forward (back) 12 inches, which is ~1 wheel rotation
        mecanumWheels.runToPosition(-ENCODER_COUNTS_PER_ROTATION, -DRIVING_POWER);
        waitOneFullHardwareCycle();
        Thread.sleep(1500); //sleeping for 1.5 seconds so motors has time to run to their position
        mecanumWheels.logEncoders();
        rotateToHeading(223);

        //go to the white line maintaining gyro headings of 225 degrees
        double headingToBeacon = 223;
        mecanumWheels.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        mecanumWheels.powerMotors(-DRIVING_POWER, 0, 0);
        double currentHeading, error, clockwiseSpeed;
        double kp = DRIVING_POWER; //experimental coefficient for proportional correction of the direction
        //alpha() is to measure the brightness.
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        while (!checkTouchObject() && colorSensorDrive.alpha() < MID_POINT_ALPHA && opModeIsActive()) {
            // keep going
            currentHeading = mecanumWheels.getGyroHeading();
            error = headingToBeacon - currentHeading;
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error value", error);
            //don't do any correction
            //if heading error < 1 degree
            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else if (Math.abs(error) >= 1 && Math.abs(error) <= 4) {
                clockwiseSpeed = kp * error/4;
            } else {
                clockwiseSpeed = DRIVING_POWER * Math.abs(error) / error;
            }

            clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            mecanumWheels.powerMotors(-DRIVING_POWER, 0, clockwiseSpeed);

            waitOneFullHardwareCycle();
            Thread.sleep(25);
        }

        mecanumWheels.powerMotors(0, 0, 0);
        waitOneFullHardwareCycle();

        if (colorSensorDrive.alpha() >= MID_POINT_ALPHA) {

            rotateToHeading(270);
            waitOneFullHardwareCycle();
            if (colorSensorDrive.alpha() < MID_POINT_ALPHA) {
                while (!checkTouchObject() && colorSensorDrive.alpha() < MID_POINT_ALPHA && opModeIsActive()) {
                    mecanumWheels.powerMotors(0, DRIVING_POWER, 0);
                    waitOneFullHardwareCycle();
                    Thread.sleep(25);
                }
                mecanumWheels.powerMotors(0,0,0);
                waitOneFullHardwareCycle();
                rotateToHeading(270);
                waitOneFullHardwareCycle();
            }

            int i = 0;
            while (!checkTouchObject() && opModeIsActive()) {
                DbgLog.msg(i + " BEACON heading " + mecanumWheels.getGyroHeading());

                if (colorSensorBeacon.blue() > 0) {
                    DbgLog.msg(i + " BEACON Detecting blue");
                    break;
                } else if (colorSensorBeacon.red() > 0) {
                    DbgLog.msg(i + " BEACON Detecting red");
                    break;
                } else {
                    DbgLog.msg(i + " BEACON No color detected");
                }

                double alpha = colorSensorDrive.alpha();
                DbgLog.msg(i + " BEACON alpha " + alpha);

                DbgLog.msg(i + " BEACON distance touch,right,left "+touchSensor.isPressed()+", "+
                        opticalDistSensorRight.getLightDetectedRaw()+", "+ opticalDistSensorLeft.getLightDetectedRaw());
                double powerDelta = (alpha - MID_POINT_ALPHA) * CORRECTION_MULTIPLIER;   //Delta = difference
                mecanumWheels.powerMotors(-0.2, 0, powerDelta);
                waitOneFullHardwareCycle();
                Thread.sleep(50);
                i++;
            }

            DbgLog.msg(" BEACON distance touch,right,left " + touchSensor.isPressed() + ", " +
                    opticalDistSensorRight.getLightDetectedRaw() + ", " + opticalDistSensorLeft.getLightDetectedRaw());

            mecanumWheels.powerMotors(0, 0, 0);
            waitOneFullHardwareCycle();
        }

/*
        //rotate robot -45 degrees counterclockwise
        rotateToHeading(45);
        Thread.sleep(1000);
        rotateToHeading(315);
        Thread.sleep(1000);
        rotateToHeading(90);
        Thread.sleep(1000);
        rotateToHeading(270);
        Thread.sleep(1000);
        rotateToHeading(0);
*/
        //todo: drive backwards until the robot runs into the white line leading to the beacon.
    }
}