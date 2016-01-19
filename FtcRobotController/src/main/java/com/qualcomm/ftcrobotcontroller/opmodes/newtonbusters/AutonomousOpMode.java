package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Autonomous mode
 * Created by Aryoman on 12/2/2015.
 * revised by Athena on 12/7/2015 - rotate method
 * revised by Alex on 12/8/2015 - going forward set distance, rotate, go until touching something
 */
public class AutonomousOpMode extends LinearOpMode {

    enum BeaconColor {red, blue, none}

    boolean blueAlliance = true;

    final static int STOP_AT_DISTANCE_READING = 50;                     //ODS reading; robot will stop at this distance from wall
    final static int MID_POINT_ALPHA_BACK = 15;                         //(0 + 30) / 2 = 15
    final static int MID_POINT_ALPHA_FRONT = 5;                         //(0 + 10) / 2 = 5
    final static double CORRECTION_MULTIPLIER = 0.012;                  //kp constant power
    final static double LINE_FOLLOW_MOTOR_POWER = -0.2;                 //power of the motor

    final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;
    //one wheel rotation covers ~12 inches
    //one tile is ~24 inches long
    //final static int TARGET_POSITION1 = 2 * ENCODER_COUNTS_PER_ROTATION;
    final static double DRIVING_POWER = 1;

    MecanumWheels mecanumWheels;

    UltrasonicSensor ultrasonicSensorRight;
    UltrasonicSensor ultrasonicSensorLeft;
    ColorSensor colorSensorFront;
    ColorSensor colorSensorBack;
    ColorSensor colorSensorBeacon;
    Servo rightButtonPusher, leftButtonPusher;
    Servo skiLiftHandleRight, skiLiftHandleLeft;
    Servo frontSweeper;
    Servo peopleDrop;
    DcMotor brush;
    DcMotor shoulderMotor;

    ElapsedTime distanceTimer;
    double lastDistance = 0;

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
        rotateToTolerance(1.5, requiredHeading, power);
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

    public boolean checkTouchObject() throws InterruptedException {
        double ultrasonicLevelTake, ultrasonicLevelRight, ultrasonicLevelLeft;
        double sum = 0;
        int n = 0;

        for (int i = 0; i < 10; i++) {
            ultrasonicLevelRight = ultrasonicSensorRight.getUltrasonicLevel();
            if (ultrasonicLevelRight < 5 || ultrasonicLevelRight > 60) {
                ultrasonicLevelRight = 500;
            }
            ultrasonicLevelLeft = ultrasonicSensorLeft.getUltrasonicLevel();
            if (ultrasonicLevelLeft < 5 || ultrasonicLevelLeft > 60) {
                ultrasonicLevelLeft = 500;
            }
            ultrasonicLevelTake = Math.min(ultrasonicLevelLeft, ultrasonicLevelRight);
            if (ultrasonicLevelTake > 499) {
                continue;
            } else {
                sum += ultrasonicLevelTake;
                n++;
            }
            waitForNextHardwareCycle();
        }
        //we assume that we are too far away to get a valid value
        if (n<1) {
            return false;
        }
        double ultrasonicLevelAverage = sum / n;

        telemetry.addData("ultrasonic level right", ultrasonicLevelAverage);
        DbgLog.msg("ULTRASONIC RIGHT " + ultrasonicLevelAverage);
        // check if the distance has changed for the last 2 secs
        if (distanceTimer.time() > 2) {
            if (lastDistance > 0 && lastDistance == ultrasonicLevelAverage) {
                return true;
            }
            lastDistance = ultrasonicLevelAverage;
            distanceTimer.reset();
        }
        return (ultrasonicLevelAverage < 18 && ultrasonicLevelAverage > 0.1);
    }

    public BeaconColor checkBeaconColor() {
        BeaconColor color;
        int blue = colorSensorBeacon.blue();
        int red = colorSensorBeacon.red();
        if (blue > red) {
            DbgLog.msg("BEACON Detecting blue");
            color = BeaconColor.blue;
        } else if (red > blue) {
            DbgLog.msg("BEACON Detecting red");
            color = BeaconColor.red;
        } else {
            DbgLog.msg("BEACON No color detected");
            color = BeaconColor.none;
        }
        return color;
    }
    public void dropPeople() throws InterruptedException {
        double currentPos = peopleDrop.getPosition();
        double newPos = currentPos;
        while (newPos >= 0 && opModeIsActive()){
            newPos = currentPos - 0.025;
            if (newPos >= 0){
                peopleDrop.setPosition(newPos);
                waitOneFullHardwareCycle();
                Thread.sleep(80);
                currentPos = peopleDrop.getPosition();
            } else {
                break;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true);
        waitOneFullHardwareCycle();

        rightButtonPusher = hardwareMap.servo.get("RightButtonPusher");
        leftButtonPusher = hardwareMap.servo.get("LeftButtonPusher");
        //0 servo setting means touch pusher isn't deployed, all the way in, didn't touch/sense anything
        //1 servo setting means touch pusher is deployed, and touched/sensed something
        leftButtonPusher.setDirection(Servo.Direction.REVERSE);
        rightButtonPusher.setPosition(0.5);
        leftButtonPusher.setPosition(0.5);

        //Zero is the dropping position, 1 is the initial position.
        peopleDrop = hardwareMap.servo.get("PeopleDrop");
        peopleDrop.setPosition(1);


        skiLiftHandleRight = hardwareMap.servo.get("SkiLiftHandleRight");
        //value 45/255 is the initial position, value 195/255 is the deployed position
        skiLiftHandleRight.setPosition(45.0 / 255);
        skiLiftHandleLeft = hardwareMap.servo.get("SkiLiftHandleLeft");
        //value 240/255 is the initial position, value 90/255 is the deployed position
        skiLiftHandleLeft.setPosition(240.0 / 255);
        frontSweeper = hardwareMap.servo.get("FrontSweeper");
        //value 200/255 is the initial position, value 100/255 is the deployed position
        frontSweeper.setPosition(125.0 / 255);

        brush = hardwareMap.dcMotor.get("Brush");
        brush.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        shoulderMotor = hardwareMap.dcMotor.get("Arm");
        shoulderMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        colorSensorFront = hardwareMap.colorSensor.get("Color Sensor Front");
        colorSensorFront.setI2cAddress(0x40);
        colorSensorFront.enableLed(true);
        colorSensorBack = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        colorSensorBeacon.setI2cAddress(0x3e);

        ultrasonicSensorRight = hardwareMap.ultrasonicSensor.get("Distance Sensor Right");
        ultrasonicSensorLeft = hardwareMap.ultrasonicSensor.get("Distance Sensor Left");

        distanceTimer = new ElapsedTime();
        distanceTimer.reset();
        lastDistance = 0;

        waitForStart();

        //we are going to raise the arm so it doesnt obstruct people rod
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        int currentArmPos = shoulderMotor.getCurrentPosition();
        shoulderMotor.setTargetPosition(currentArmPos-1350);
        shoulderMotor.setPower(0.3);

        // deploy sweeper
        frontSweeper.setPosition(105 / 255d);
        waitOneFullHardwareCycle();
        Thread.sleep(1500);
        brush.setPower(1);
        waitOneFullHardwareCycle();
        Thread.sleep(500);

        //we dont need power to hold arm at 90 degrees
        shoulderMotor.setPower(0.0);

        //we adjust people rod position so that people have time to settle and
        // dont swing too much when we drop them
        peopleDrop.setPosition(0.25);


        /*
         We are assuming gyro calibration with robot pointing backward was performed right before.
         At the start of autonomous mode, we are visually pointing robot (using the arm)
         to the white line (just past its start)
         */
        double headingToBeaconZone = mecanumWheels.getGyroHeading();

        //go to the white line maintaining gyro headings to beacon

        mecanumWheels.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        mecanumWheels.powerMotors(DRIVING_POWER, 0, 0);
        double currentHeading, error, clockwiseSpeed;
        double kp = DRIVING_POWER; //experimental coefficient for proportional correction of the direction
        //alpha() is to measure the brightness.
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        int i = 0;
        while (colorSensorFront.alpha() < MID_POINT_ALPHA_FRONT && opModeIsActive()) {
            // keep going
            currentHeading = mecanumWheels.getGyroHeading();
            error = headingToBeaconZone - currentHeading;
            //telemetry.addData("Current Heading", currentHeading);
            //telemetry.addData("Error value", error);
            //don't do any correction
            //if heading error < 1 degree
            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else if (Math.abs(error) >= 1 && Math.abs(error) <= 4) {
                clockwiseSpeed = kp * error / 4;
            } else {
                clockwiseSpeed = DRIVING_POWER * Math.abs(error) / error;
            }
            clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            //DbgLog.msg(i + " clockwise speed "+clockwiseSpeed);
            mecanumWheels.powerMotors(DRIVING_POWER, 0, clockwiseSpeed);

            waitOneFullHardwareCycle();
            //Thread.sleep(25);
            i++;
        }
        //mecanumWheels.powerMotors(0, 0, 0);
        //waitOneFullHardwareCycle();
        //Thread.sleep(100);

        if (!checkTouchObject() && opModeIsActive()) {
            // move debris out out of the way passed the line
            mecanumWheels.powerMotors(DRIVING_POWER, 0, 0);
            waitOneFullHardwareCycle();
            long startpos = Math.abs(mecanumWheels.motorFrontLeft.getCurrentPosition());
            long endpos = startpos;
            while (endpos - startpos < 0.5 * (2 * 1140)) {
                waitOneFullHardwareCycle();
                endpos = Math.abs(mecanumWheels.motorFrontLeft.getCurrentPosition());
            }
            mecanumWheels.powerMotors(0, 0, 0);
            waitOneFullHardwareCycle();
            Thread.sleep(100);

            // move back to the line
            mecanumWheels.powerMotors(-0.5, 0, 0);
            while (colorSensorFront.alpha() < MID_POINT_ALPHA_FRONT && opModeIsActive()) {
                waitOneFullHardwareCycle();
            }
            mecanumWheels.powerMotors(0, 0, 0);
            waitOneFullHardwareCycle();
            Thread.sleep(100);

            // rotate to the beacon
            double headingToBeacon = blueAlliance ? 270 : 90;
            rotateToHeading(headingToBeacon);
            waitOneFullHardwareCycle();
            //Thread.sleep(1000);

            // undeploy sweeper
            brush.setPower(0);
            waitOneFullHardwareCycle();
            frontSweeper.setPosition(125 / 255d);
            waitOneFullHardwareCycle();

            // shift to the left to find line
            if (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && !checkTouchObject() && opModeIsActive()) {
                mecanumWheels.powerMotors(0, 0.8, 0);
                while (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && !checkTouchObject() && opModeIsActive()) {
                    waitOneFullHardwareCycle();
                }
                mecanumWheels.powerMotors(0, 0, 0);
                waitOneFullHardwareCycle();
                Thread.sleep(100);
                // if we overshot, move back
                if (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && !checkTouchObject() && opModeIsActive()) {
                    telemetry.addData("Overshot", colorSensorBack.alpha());
                    mecanumWheels.powerMotors(0, -0.8, 0);
                    waitOneFullHardwareCycle();
                    while (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && !checkTouchObject() && opModeIsActive()) {
                        waitOneFullHardwareCycle();
                    }
                    mecanumWheels.powerMotors(0, 0, 0);
                    waitOneFullHardwareCycle();
                    Thread.sleep(100);
                }

            }
            //Thread.sleep(1000);

            // last shift to the right to make sure it's on the right side of the line.
            mecanumWheels.powerMotors(0, -0.2, 0);
            waitOneFullHardwareCycle();
            Thread.sleep(100);

            // follow line
            i = 0;
            while (!checkTouchObject() && opModeIsActive()) {

                double alpha = colorSensorBack.alpha();
                //DbgLog.msg(i + " BEACON alpha " + alpha);
                //DbgLog.msg(i + " BEACON distance " + ultrasonicSensorRight.getUltrasonicLevel());

                double powerDelta = (alpha - MID_POINT_ALPHA_BACK) * CORRECTION_MULTIPLIER;   //Delta = difference
                mecanumWheels.powerMotors(-0.2, 0, powerDelta);
                waitOneFullHardwareCycle();
                i++;
            }
            mecanumWheels.powerMotors(0, 0, 0);
            waitOneFullHardwareCycle();
            Thread.sleep(100);
            DbgLog.msg(i + " BEACON distance " + ultrasonicSensorRight.getUltrasonicLevel());
            DbgLog.msg(i + " BEACON heading " + mecanumWheels.getGyroHeading());

            //Trying several times to find the color, in case it is not initially detected.
            BeaconColor color = BeaconColor.none;
            for (int c = 0; c < 10; c++) {
                color = checkBeaconColor();
                waitOneFullHardwareCycle();
                if (color != BeaconColor.none) {
                    break;
                }
            }

            if (color != BeaconColor.none && opModeIsActive()) {
                dropPeople();
                waitOneFullHardwareCycle();
                BeaconColor allianceColor = blueAlliance ? BeaconColor.blue : BeaconColor.red;
                //robot has detected the beacon's color
                //our beacon color sensor is on the right (when front of robot is facing forward)
                if (color == allianceColor) {
                    //deploy the right button pusher
                    rightButtonPusher.setPosition(1);
                    waitOneFullHardwareCycle();
                    Thread.sleep(2500);
                    if (opModeIsActive()) {
                        rightButtonPusher.setPosition(0);
                    }
                } else {
                    //deploy the left button pusher
                    leftButtonPusher.setPosition(1);
                    waitOneFullHardwareCycle();
                    Thread.sleep(2500);
                    if (opModeIsActive()) {
                        leftButtonPusher.setPosition(0);
                    }
                }
                // move backwards and drop people
                mecanumWheels.powerMotors(0.7, 0, 0);
                waitOneFullHardwareCycle();
                Thread.sleep(600);

                mecanumWheels.powerMotors(0, 0, 0);
                waitOneFullHardwareCycle();
                Thread.sleep(100);

                // move sideways.
                mecanumWheels.powerMotors(0, -1d, 0);
                waitOneFullHardwareCycle();
                Thread.sleep(2500);

                mecanumWheels.powerMotors(0, 0, 0);
                waitOneFullHardwareCycle();
                Thread.sleep(100);
            }
            waitOneFullHardwareCycle();

            peopleDrop.setPosition(0.5); // almost vertical position
            waitOneFullHardwareCycle();
        }

    }

}