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
public class MecanumWheels {

    // this variable is for gyro drift mitigation;
    // we store the heading of the pointed robot in this variable when running GyroCalibration op mode;
    // on AutomousOpMode init we calculate the difference between current and saved heading,
    // it gives us gyro drift, which we store as gyroForwardOffset.
    static double pointedHeading;

    //defining the 4 motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;
    GyroSensor sensorGyro;
    Telemetry telemetry;
    boolean useGyro;
    double gyroForwardOffset; //between 0 and 359 inclusive


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
            // default - forward config
            gyroForwardOffset = 0;
        }
    }

    public void backwardSetup() {
        resetEncoders();
        calibrateGyro();
        gyroForwardOffset = 180;

    }

    public void forwardSetup() {
        resetEncoders();
        calibrateGyro();
        gyroForwardOffset = 0;

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
            }
        }

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
        DbgLog.msg("Gyro heading offset " + gyroForwardOffset);
    }


    public double getHeadingError(double requiredHeading) {

        double headingError = sensorGyro.getHeading()-requiredHeading;
        if (headingError > 180) {
            headingError -= 360;
        } else if (headingError < -180) {
            headingError +=360;
        }
        return headingError;
    }


    public void adjustGyroHeading() {
        gyroForwardOffset = getHeadingError(pointedHeading); // returned range -180 - +180
        telemetry.addData("Gyro heading error", gyroForwardOffset);
        // gyroForwardOffset range should be 0-360
        if (gyroForwardOffset<0) { gyroForwardOffset +=360; }
        DbgLog.msg("Gyro heading offset " + gyroForwardOffset + " deg");
    }

    public double getGyroHeading() {
        double heading = sensorGyro.getHeading() - gyroForwardOffset;
        if (heading < 0) {
            heading += 360;
        }

        telemetry.addData("heading", heading);
        return heading;
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

            //returns heading corrected for drift or driver offset
            double headingDegrees = getGyroHeading();
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
            //telemetry.addData("Fld Forward,Right", (int) (fieldForward * 100) + ", " + (int) (fieldRight * 100));
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
        /*
        telemetry.addData("Robot Forward,Right,Clockwise", (int) (forward * 100) +
                ", " + (int) (right * 100)+
                ", " + (int) (clockwise * 100));
        */
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


        String message = "Reset encoders did not complete";

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            message += " " + e.getMessage();
            DbgLog.msg(message);
            telemetry.addData("ERROR1", message);
        }

        while (Math.abs(motorFrontLeft.getCurrentPosition()) > 0 ||
                Math.abs(motorFrontRight.getCurrentPosition()) > 0 ||
                Math.abs(motorRearLeft.getCurrentPosition()) > 0 ||
                Math.abs(motorRearRight.getCurrentPosition()) > 0) {
            DbgLog.msg("-PosFL,FR,RL,RR " +
                    motorFrontLeft.getCurrentPosition() + "," +
                    motorFrontRight.getCurrentPosition() + "," +
                    motorRearLeft.getCurrentPosition() + "," +
                    motorRearRight.getCurrentPosition());
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                message += " " + e.getMessage();
                DbgLog.msg(message);
                telemetry.addData("ERROR1", message);
            }

        }

        logEncoders();
        setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void logEncoders() {

        DbgLog.msg("PosFL,FR,RL,RR " + motorFrontLeft.getCurrentPosition() + "," +
                motorFrontRight.getCurrentPosition() + "," +
                motorRearLeft.getCurrentPosition() + "," +
                motorRearRight.getCurrentPosition());
        telemetry.addData("PosFL,FR,RL,RR", motorFrontLeft.getCurrentPosition() + "," +
                motorFrontRight.getCurrentPosition() + "," +
                motorRearLeft.getCurrentPosition() + "," +
                motorRearRight.getCurrentPosition());

    }

    public void runToPosition(int counts, double power) {

        DbgLog.msg("Counts to Target Pos " + counts);

        int flTargetPos = motorFrontLeft.getCurrentPosition()+counts;
        motorFrontLeft.setTargetPosition(flTargetPos);
        int frTargetPos = motorFrontRight.getCurrentPosition()+counts;
        motorFrontRight.setTargetPosition(frTargetPos);
        int rlTargetPos = motorRearLeft.getCurrentPosition()+counts;
        motorRearLeft.setTargetPosition(rlTargetPos);
        int rrTargetPos = motorRearRight.getCurrentPosition()+counts;
        motorRearRight.setTargetPosition(rrTargetPos);
        //DbgLog.msg("Target PosFL,FR,RL,RR " + flTargetPos + "," + frTargetPos + "," + rlTargetPos + "," + rrTargetPos);

        setRunMode(DcMotorController.RunMode.RUN_TO_POSITION);

        powerMotors(power, 0, 0);
        // this method must be followed by a delay for completion
    }

}