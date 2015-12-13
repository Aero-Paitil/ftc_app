package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
    final static int TARGET_POSITION1 = 2 * ENCODER_COUNTS_PER_ROTATION;
    final static double DRIVING_POWER = -0.5;

    MecanumWheels mecanumWheels;

    /*
    Arm arm;
    Brushes brushes;
    Servo beacon6;


    ColorSensor colorSensorBeacon;

    */
    OpticalDistanceSensor opticalDistSensorRight;
    OpticalDistanceSensor opticalDistSensorLeft;
    ColorSensor colorSensorDrive;

    // gyro heading is from 0 to 359
    public void rotateToHeading(double requiredHeading) throws InterruptedException {

        mecanumWheels.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


        double MIN_ROTATE_POWER = 0.2;

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
        return opticalDistSensorLeft.getLightDetectedRaw() > 10 || opticalDistSensorRight.getLightDetectedRaw() > 10;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true);
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");

        /*
        mecanumWheels.resetEncoders();

        brushes = new Brushes(hardwareMap, telemetry);

        arm = new Arm(hardwareMap, telemetry);

        beacon6 = hardwareMap.servo.get("Beacon6");
        beacon6.setPosition(0.5);

        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        */
        opticalDistSensorRight = hardwareMap.opticalDistanceSensor.get("Optical Distance Right");
        opticalDistSensorLeft = hardwareMap.opticalDistanceSensor.get("Optical Distance Left");

        waitForStart();


        //to be able to use field coordinates in the following driving mode
        // our robot must be faced forward in the beginning

        //assuming we are starting at the tile next to the mountain

        /*
        // go forward 2*24 inches
        mecanumWheels.runToPosition(TARGET_POSITION1, DRIVING_POWER);

        //release brushes
        brushes.autonomousUndockBrushes();

        //undock arm
        arm.autonomousUndockArm();
        */

        mecanumWheels.runToPosition(ENCODER_COUNTS_PER_ROTATION, 0.4);
        waitOneFullHardwareCycle();
        Thread.sleep(2000);
        rotateToHeading(225);

        double headingToBeacon = 225;
        mecanumWheels.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        mecanumWheels.powerMotors(-0.2, 0, 0);
        double currentHeading, error, clockwiseSpeed;
        double kp = 0.05; //experimental coefficient for proportional correction of the direction
        while (!checkTouchObject() && colorSensorDrive.alpha() < MID_POINT_ALPHA) {
            // keep going
            currentHeading = mecanumWheels.getGyroHeading();
            error = headingToBeacon - currentHeading;
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error value", error);
            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else if (Math.abs(error) >= 1 && Math.abs(error) <= 5) {
                clockwiseSpeed = kp * error;
            } else {
                clockwiseSpeed = 0.2 * Math.abs(error) / error;
            }
            mecanumWheels.powerMotors(-0.2, 0, clockwiseSpeed);


            waitOneFullHardwareCycle();
            Thread.sleep(50);
        }

        mecanumWheels.powerMotors(0, 0, 0);
        waitOneFullHardwareCycle();
        rotateToHeading(270);
        Thread.sleep(200);
        while (!checkTouchObject()) {
            double alpha = colorSensorDrive.alpha();
            telemetry.addData("Alpha", alpha);
            double powerDelta = (alpha - MID_POINT_ALPHA) * CORRECTION_MULTIPLIER;   //Delta = difference
            mecanumWheels.powerMotors(-0.2, 0, powerDelta);
            waitOneFullHardwareCycle();
            Thread.sleep(50);
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
