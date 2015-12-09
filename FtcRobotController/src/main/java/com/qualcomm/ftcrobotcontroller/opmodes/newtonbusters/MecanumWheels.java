package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * This class is controlling the mecanum wheels driving platform. It can be used with or without the gyro sensor.
 * Created by Aryoman on 11/24/2015.
 */
public class MecanumWheels { //defining the 4 motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;
    GyroSensor sensorGyro;
    Telemetry telemetry;
    boolean useGyro;
    double gyroForwardOffset;


    public MecanumWheels(HardwareMap hardwareMap, Telemetry telemetry, boolean useGyro) {
        this.telemetry = telemetry;
        this.useGyro = useGyro;

        //"initializing" the motors
        motorFrontLeft = hardwareMap.dcMotor.get("DC1");
        motorFrontRight = hardwareMap.dcMotor.get("DC2");
        motorRearLeft = hardwareMap.dcMotor.get("DC3");
        motorRearRight = hardwareMap.dcMotor.get("DC4");

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);

        if (useGyro) {
            sensorGyro = hardwareMap.gyroSensor.get("Gyro Sensor");
            calibrateGyro();
            gyroForwardOffset = 0;
        }
    }

    public void calibrateGyro() {
        // calibrate the gyro.
        sensorGyro.calibrate();

        // wait for calibration to start
        String message = "Gyro has not been fully calibrated";
        while (!sensorGyro.isCalibrating()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                message += " " + e.getMessage();
                DbgLog.msg(message);
                telemetry.addData("ERROR1", message);
                break;
            }        }

        // make sure the gyro is calibrated.
        long elapsed = System.currentTimeMillis();
        while (sensorGyro.isCalibrating()) {
            try {
                Thread.sleep(50);
                //waitOneFullHardwareCycle();
            } catch (InterruptedException e) {
                message += " " + e.getMessage();
                DbgLog.msg(message);
                telemetry.addData("ERROR2", message);

                break;
            }
            if (System.currentTimeMillis() - elapsed > 5000 /*5 second*/) {
                DbgLog.msg(message);
                break;
            }
        }
        //sensorGyro.resetZAxisIntegrator();
        telemetry.addData("Gyro calibrated", sensorGyro.status());
    }

    public void resetGyroHeading() {
        //sensorGyro.resetZAxisIntegrator();
        gyroForwardOffset = sensorGyro.getHeading();
        telemetry.addData("Gyro heading new offset", gyroForwardOffset);
    }

    public void powerMotors(double forward, double right, double clockwise) {
        powerMotors(forward, right, clockwise, false);
    }

    public void powerMotors(double forward, double right, double clockwise, boolean fieldCoordinates) {
        if (fieldCoordinates) {
            if (!useGyro) {
                telemetry.addData("ERROR", "Field coordinates require Gyro.");
                return;
            }
            //get the angle from field's forward to robot's forward
            double gyroHeading = sensorGyro.getHeading();
            //adjusting for current firld forward direction
            double headingDegrees = gyroHeading - gyroForwardOffset;
            if (headingDegrees < 0) {
                headingDegrees += 360;
            }
            //headingDegrees is clockwise
            double myheading = -Math.PI * headingDegrees / 180.0;

            //We are using the joystick values in the driver's perspective (field coordinates).
            //this link gives the formula: http://www.mathematics-online.org/inhalt/aussage/aussage444/
            double fieldForward = forward; //field coordinates
            double fieldRight = right; //field coordinates
            right = Math.cos(myheading) * fieldRight + Math.sin(myheading) * fieldForward; //robot coordinates
            forward = -Math.sin(myheading) * fieldRight + Math.cos(myheading) * fieldForward; //robot coordinates

            // Telemetry - all doubles are scaled to (-100, 100)
            telemetry.addData("Heading", headingDegrees + " deg");
            telemetry.addData("Fld Forward,Right", (int) (fieldForward * 100) + ", " + (int) (fieldRight * 100));
        }

        //add deadband so you don't strafe when you don't want to. A deadband is essentially if you want to go to the right,
        //and the joystick is 7 degrees short of 90 degrees, instead of having the robot slowly creep forward, the robot will
        //ignore the small degrees and just go to the right.
        //todo adjust the deadband
        if ((right > -0.1) && (right < 0.1)) right = 0;
        if ((forward > -0.1) && (forward < 0.1)) forward = 0;


        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        /*this is scaling the motor power. Since our motors work on a scale between -1 and 1, and when we input
        values into the controller, they can be greater than one. We want to make sure that all values are between 1 and -1.
         we do that by first figuring out what the maximum value is, and then dividing all the numbers by the max value. Therefore
        the max power will be 1 (or -1, if we are going in reverse) and the other powers will be less than one. */
        //todo find the maximum absolute value for any motor scaled power
        double max = Math.abs(front_left);
        max = Math.max(Math.abs(front_right), max);
        max = Math.max(Math.abs(rear_left), max);
        max = Math.max(Math.abs(rear_right), max);

        if (max > 1) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }

        /* assigning the motors the scaled powers that we just calculated in the step above. */
        motorFrontLeft.setPower(front_left);
        motorFrontRight.setPower(front_right);
        motorRearLeft.setPower(rear_left);
        motorRearRight.setPower(rear_right);

        // Telemetry - all doubles are scaled to (-100, 100)
        telemetry.addData("Robot Forward,Right,Clockwise", (int) (forward * 100) +
                ", " + (int) (right * 100)+
                ", " + (int) (clockwise * 100));
        telemetry.addData("DC1,2,3,4", (int) (front_left * 100) + ", " +
                (int) (front_right * 100) + ", " +
                (int) (rear_left * 100) + ", " +
                (int) (rear_right * 100));
    }

    public void setRunMode(DcMotorController.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorRearLeft.setMode(runMode);
        motorRearRight.setMode(runMode);
    }


    //-----------------------
    //AUTONOMOUS MODE METHODS
    //-----------------------



    public void resetEncoders() {
        setRunMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void runToPosition(int counts, double power) {
        telemetry.addData("Counts", counts);
        telemetry.addData("PosFL", motorFrontLeft.getCurrentPosition());
        telemetry.addData("PosFR", motorFrontRight.getCurrentPosition());
        int flTargetPos = motorFrontLeft.getCurrentPosition()+counts;
        int frTargetPos = motorFrontRight.getCurrentPosition()+counts;
        int rlTargetPos = motorRearLeft.getCurrentPosition()+counts;
        int rrTargetPos = motorRearRight.getCurrentPosition()+counts;
        motorFrontLeft.setTargetPosition(flTargetPos);
        motorFrontRight.setTargetPosition(frTargetPos);
        motorRearLeft.setTargetPosition(rlTargetPos);
        motorRearRight.setTargetPosition(rrTargetPos);

        motorFrontLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRearRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearLeft.setPower(power);
        motorRearRight.setPower(power);
        Utils.delay(0.05);
        while (motorFrontLeft.getCurrentPosition()-flTargetPos>5 ||
                motorFrontRight.getCurrentPosition()-frTargetPos>5 ||
                motorRearLeft.getCurrentPosition()-rlTargetPos>5 ||
                motorRearRight.getCurrentPosition()-rrTargetPos>5){
            telemetry.addData("PosFL", motorFrontLeft.getCurrentPosition());
            telemetry.addData("PosFR", motorFrontRight.getCurrentPosition());
            Utils.delay(0.05);
        }

    }

    public double getGyroHeading() {
        double heading = sensorGyro.getHeading();
        telemetry.addData("heading", heading);
        return heading;
    }


}