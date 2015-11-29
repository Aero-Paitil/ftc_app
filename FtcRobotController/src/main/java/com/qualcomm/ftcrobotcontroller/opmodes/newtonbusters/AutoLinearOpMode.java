package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * 11/29/15.
 */
public class AutoLinearOpMode extends LinearOpMode {
    // To do - change to use MecanumWheels class???
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;
    ColorSensor colorSensorDrive;
    ColorSensor colorSensorBeacon;
    OpticalDistanceSensor opticalDistSensor;

    // The following values needs to be adjusted for line follow
    final static int STOP_AT_DISTANCE_READING = 50;                     //ODS reading; robot will stop at this distance from wall
    final static int MID_POINT_ALPHA = 20;                              //(0 + 40) / 2 = 20
    final static double CORRECTION_MULTIPLIER = 0.012;                  //kp konstant power
    final static double LINE_FOLLOW_MOTOR_POWER = -0.2;                 //power of the motor
    /**
     * Follow line to beacon, util the distance sensor get to reading 50
     */
    void followLineToBeacon() throws InterruptedException {
        motorFrontLeft.setPower(LINE_FOLLOW_MOTOR_POWER);
        motorFrontRight.setPower(LINE_FOLLOW_MOTOR_POWER);
        motorRearLeft.setPower(LINE_FOLLOW_MOTOR_POWER);
        motorRearRight.setPower(LINE_FOLLOW_MOTOR_POWER);

        while (opticalDistSensor.getLightDetectedRaw() < STOP_AT_DISTANCE_READING) {
            //colorSensorDrive.alpha() - MID_POINT_ALPHA = error
            //CORRECTION_MULTIPLIER = Kp                                //proportionality constant, slope
            //powerDelta = turn
            //turn = error * Kp
            double powerDelta = (colorSensorDrive.alpha() - MID_POINT_ALPHA) * CORRECTION_MULTIPLIER;   //Delta = difference
            // To do - make sure the power is between -1 to 1 ...
            motorFrontLeft.setPower(LINE_FOLLOW_MOTOR_POWER + powerDelta);
            motorRearLeft.setPower(LINE_FOLLOW_MOTOR_POWER + powerDelta);
            motorFrontRight.setPower(LINE_FOLLOW_MOTOR_POWER - powerDelta);
            motorRearRight.setPower(LINE_FOLLOW_MOTOR_POWER - powerDelta);
            waitOneFullHardwareCycle();
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorRearLeft.setPower(0);
        motorRearRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //"initializing" the motors
        motorFrontLeft = hardwareMap.dcMotor.get("DC1");
        motorFrontRight = hardwareMap.dcMotor.get("DC2");
        motorRearLeft = hardwareMap.dcMotor.get("DC3");
        motorRearRight = hardwareMap.dcMotor.get("DC4");

        // These two returns the same sensor currently until we change one of them to a different
        // I2C address.
        //
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        opticalDistSensor = hardwareMap.opticalDistanceSensor.get("Optical Distance");

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);

        // 1. Navigate to the white line using Gyro, and stop when color sensor finds white line
        // 2. Rotate to appropriate angle using Gyro while maintain on white line
        // 3. Follow white line until close/reach the beacon by touch sensor or distance sensor
        followLineToBeacon();
        // 4. Read the color of beacon, touch the right button
        // 5. Use gyro to parking (mountain or parking lot)

    }
}
