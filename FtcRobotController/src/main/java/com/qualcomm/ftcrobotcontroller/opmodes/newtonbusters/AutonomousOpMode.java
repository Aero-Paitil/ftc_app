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
public abstract class AutonomousOpMode extends LinearOpMode {

    enum BeaconColor {red, blue, none}

    boolean blueAlliance = isBlueAlliance();

    //final static int STOP_AT_DISTANCE_READING = 50;                   //ODS reading; robot will stop at this distance from wall
    final static int MID_POINT_ALPHA_BACK = 15;                         //(0 + 30) / 2 = 15
    final static int MID_POINT_ALPHA_FRONT = 5;                         //(0 + 10) / 2 = 5
    final static double CORRECTION_MULTIPLIER = 1.0;                    //kp constant power
    final static double LINE_FOLLOW_MOTOR_POWER = -0.15;                //power of the motor

    final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;
    //one wheel rotation covers ~12 inches
    //one tile is ~24 inches long
    //final static int TARGET_POSITION1 = 2 * ENCODER_COUNTS_PER_ROTATION;
    final static double DRIVING_POWER = 1;

    final static public double SWEEPER_INITIAL_POS = 1.0;
    final static public double SWEEPER_UNDEPLOYED_POS = 0.85;
    final static public double SWEEPER_DEPLOYED_POS = 0.07;

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
    Servo wheelProtectionPort5, wheelProtectionPort6;
    Servo spool1;
    Servo spool2;
    DcMotor brush;
    DcMotor shoulderMotor;

    ElapsedTime distanceTimer;
    double lastDistance = 0;

    abstract boolean isBlueAlliance();

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
        rotateToTolerance(2.5, requiredHeading, power * 0.75);
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
        double lastHeading = headingDelta;

        if (headingDelta > tolerance) {
            // power motors
            mecanumWheels.powerMotors(0, 0, power);
            waitOneFullHardwareCycle();
            // if current gyro heading is not close enough to the required heading
            // wait and check again
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (headingDelta > tolerance && headingDelta <= lastHeading + 1 && opModeIsActive()) {
                checkTimeout(timer, 10);
                waitForNextHardwareCycle();
                lastHeading = headingDelta;
                headingDelta = getHeadingDelta(requiredHeading);
                //telemetry.addData("delta", headingDelta);
            }
        }
    }

    public boolean checkTouchObject() throws InterruptedException {
        double ultrasonicLevelTake, ultrasonicLevelRight, ultrasonicLevelLeft;
        double sum = 0;
        int n = 0;

        for (int i = 0; i < 10; i++) {
            ultrasonicLevelRight = ultrasonicSensorRight.getUltrasonicLevel();
            if (ultrasonicLevelRight <= 5 || ultrasonicLevelRight > 60) {
                ultrasonicLevelRight = 500;
            }
            ultrasonicLevelLeft = ultrasonicSensorLeft.getUltrasonicLevel();
            if (ultrasonicLevelLeft <= 5 || ultrasonicLevelLeft > 60) {
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
        if (n < 1) {
            return false;
        }
        double ultrasonicLevelAverage = sum / n;

        DbgLog.msg("ULTRASONIC RIGHT " + ultrasonicLevelAverage);
        // check if the distance has changed for the last 2 secs
        if (distanceTimer.time() > 2) {
            if (lastDistance > 0 && lastDistance < 25 && lastDistance == ultrasonicLevelAverage) {
                return true;
            }
            lastDistance = ultrasonicLevelAverage;
            distanceTimer.reset();
        }
        telemetry.addData("ultrasonicLevelAverage: ", ultrasonicLevelAverage);
        return (ultrasonicLevelAverage < 15 && ultrasonicLevelAverage > 0.1);
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
        while (newPos >= 0 && opModeIsActive()) {
            newPos = currentPos - 0.025;
            if (newPos >= 0) {
                peopleDrop.setPosition(newPos);
                waitOneFullHardwareCycle();
                sleep(80);
                currentPos = peopleDrop.getPosition();
            } else {
                break;
            }
        }
    }

    public void telemetry() {
        telemetry.addData("Beacon", "" + colorSensorBeacon.red() + "/" + colorSensorBeacon.green() + "/" +
                colorSensorBeacon.blue() + "   " +
                " at " + colorSensorBeacon.getI2cAddress() + " " + colorSensorBeacon.getConnectionInfo());
        telemetry.addData("Drive Back", colorSensorBack.alpha() +
                " at " + colorSensorBack.getI2cAddress() + " " + colorSensorBack.getConnectionInfo());
        telemetry.addData("Drive Front", colorSensorFront.alpha() +
                " at " + colorSensorFront.getI2cAddress() + " " + colorSensorFront.getConnectionInfo());
        telemetry.addData("Ultrasonic Right", ultrasonicSensorRight.getUltrasonicLevel());
        telemetry.addData("Ultrasonic Left", ultrasonicSensorLeft.getUltrasonicLevel());
        telemetry.addData("heading", mecanumWheels.getGyroHeading());
    }

    public void runForDistance(double power, double inches) throws InterruptedException {
        double diffCounts = inches * ENCODER_COUNTS_PER_ROTATION / 12.0;
        long startpos = mecanumWheels.motorFrontLeft.getCurrentPosition();
        double diff = 0;
        mecanumWheels.powerMotors(power, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (diff <= diffCounts && timer.time() < 5) {
            waitOneFullHardwareCycle();
            diff = Math.abs(mecanumWheels.motorFrontLeft.getCurrentPosition() - startpos);
        }

        stopMoving();
    }

    public void stopMoving() throws InterruptedException {
        mecanumWheels.powerMotors(0, 0, 0);
        waitOneFullHardwareCycle();
        sleep(100);
    }

    public void checkTimeout(ElapsedTime timer, int timeoutSecs) throws InterruptedException {
        if (timer.time() > timeoutSecs) {
            stopMoving();
            brush.setPower(0);
            waitOneFullHardwareCycle();
            peopleDrop.setPosition(0.5);
            waitOneFullHardwareCycle();
            while (opModeIsActive()) {
                telemetry();
                telemetry.addData("Robot STOPPED", "timeout of " + timeoutSecs + " exceeded");
                waitOneFullHardwareCycle();
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
        frontSweeper.setPosition(SWEEPER_INITIAL_POS);

        wheelProtectionPort5 = hardwareMap.servo.get("WheelProtectionLeft");
        wheelProtectionPort5.setPosition(0.5);
        wheelProtectionPort6 = hardwareMap.servo.get("WheelProtectionRight");
        wheelProtectionPort6.setPosition(0.5);

        spool1 = hardwareMap.servo.get("Spool1");
        spool1.setPosition(0.5);
        spool2 = hardwareMap.servo.get("Spool2");
        spool2.setPosition(0.5);


        brush = hardwareMap.dcMotor.get("Brush");
        brush.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        shoulderMotor = hardwareMap.dcMotor.get("Arm");
        shoulderMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();

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

        while (!opModeIsActive()) {
            telemetry();
            waitOneFullHardwareCycle();
            sleep(250);
            mecanumWheels.adjustGyroHeading();
        }

        waitForStart();

        /*
         Before autonomous mode, we are calibrating gyro and visually pointing the robot
         (using the arm as a pointing aid) to the white line (just past its start)
         */
        double headingToBeaconZone = mecanumWheels.getGyroHeading();

        //we are going to raise the arm so it doesn't obstruct people rod
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        int currentArmPos = shoulderMotor.getCurrentPosition();
        int targetArmPos = currentArmPos - 1350;
        shoulderMotor.setTargetPosition(targetArmPos);
        shoulderMotor.setPower(0.3);
        waitOneFullHardwareCycle();

        //the goal of this code is to clear the debris immediately infront of the robot

        //we drive forward for 0.5 seconds at half power to move the debris away from the
        //front of the robot.
        runForDistance(0.3, 12);

        // move back to the line
        runForDistance(-0.3, 6);


        //This code raises the wheel protection to make room for the sweeper
        ElapsedTime servoTimer = new ElapsedTime();
        wheelProtectionPort5.setPosition(0.8);
        wheelProtectionPort6.setPosition(0.75);
        waitOneFullHardwareCycle();
        servoTimer.reset();
        while (servoTimer.time() < 2.5) {
            waitOneFullHardwareCycle();
            sleep(100);
        }
        wheelProtectionPort5.setPosition(0.5);
        wheelProtectionPort6.setPosition(0.5);
        waitOneFullHardwareCycle();

        //checking to see if the arm is raised. If not, we try
        //to raise the arm again.
        currentArmPos = shoulderMotor.getCurrentPosition();
        if (Math.abs(targetArmPos - currentArmPos) > 100) {
            shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(targetArmPos);
            shoulderMotor.setPower(0.3);
            waitOneFullHardwareCycle();
        }


        // deploy sweeper
        frontSweeper.setPosition(SWEEPER_DEPLOYED_POS);
        waitOneFullHardwareCycle();
        //sleep(1500);
        sleep(500);  //Start brush rotation before the bar is down
        brush.setPower(1);
        waitOneFullHardwareCycle();
        sleep(1000);

        //we dont need power to hold arm at 90 degrees
        shoulderMotor.setPower(0.0);

        //we adjust people rod position so that people have time to settle and
        // dont swing too much when we drop them
        peopleDrop.setPosition(0.25);


        //go to the white line maintaining gyro headings to beacon
        mecanumWheels.setRunMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        mecanumWheels.powerMotors(DRIVING_POWER, 0, 0);
        double currentHeading, error, clockwiseSpeed;
        double kp = DRIVING_POWER; //experimental coefficient for proportional correction of the direction
        //alpha() is to measure the brightness.
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        int i = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double alpha = colorSensorFront.alpha();
        while (alpha < MID_POINT_ALPHA_FRONT && opModeIsActive()) {
            checkTimeout(timer, 12);
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
            //sleep(25);
            alpha = colorSensorFront.alpha();
            i++;
        }

        if (opModeIsActive()) {
            // move debris out out of the way passed the line
            runForDistance(DRIVING_POWER, 6);


            // move back to the line
            long startpos = mecanumWheels.motorFrontLeft.getCurrentPosition();
            long diff = 0;
            mecanumWheels.powerMotors(-0.5, 0, 0);
            timer.reset();
            while (colorSensorFront.alpha() < MID_POINT_ALPHA_FRONT && (diff < 0.5 * (ENCODER_COUNTS_PER_ROTATION) + 100) && opModeIsActive()) {
                waitOneFullHardwareCycle();
                diff = Math.abs(mecanumWheels.motorFrontLeft.getCurrentPosition() - startpos);
                checkTimeout(timer, 5);
            }
            stopMoving();

            // rotate to the beacon
            double headingToBeacon = blueAlliance ? 270 : 90;
            rotateToHeading(headingToBeacon);
            waitOneFullHardwareCycle();
            //sleep(1000);

            // undeploy sweeper
            brush.setPower(0);
            waitOneFullHardwareCycle();
            frontSweeper.setPosition(SWEEPER_UNDEPLOYED_POS);
            waitOneFullHardwareCycle();

            // shift to find line
            if (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && opModeIsActive()) {
                double strafePower = blueAlliance ? 0.8 : -0.8;
                mecanumWheels.powerMotors(0, strafePower, 0);
                timer.reset();
                while (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && opModeIsActive()) {
                    waitOneFullHardwareCycle();
                    checkTimeout(timer, 5);
                }
                stopMoving();

                // if we overshot, move back
                if (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && opModeIsActive()) {
                    telemetry.addData("Overshot", colorSensorBack.alpha());
                    mecanumWheels.powerMotors(0, -strafePower, 0);
                    waitOneFullHardwareCycle();
                    timer.reset();
                    while (colorSensorBack.alpha() < MID_POINT_ALPHA_BACK && opModeIsActive()) {
                        waitOneFullHardwareCycle();
                        checkTimeout(timer, 3);
                    }
                    stopMoving();
                }

            }
            //sleep(1000);

            // last shift to the right to make sure it's on the right side of the line.
            double tiltPower = blueAlliance ? -0.3 : -0.3;
            //mecanumWheels.powerMotors(0, tiltPower, 0);
            //waitOneFullHardwareCycle();
            //sleep(100);

            if (colorSensorBack.alpha() > MID_POINT_ALPHA_BACK && opModeIsActive()) {
                mecanumWheels.powerMotors(0, tiltPower, 0);
                waitOneFullHardwareCycle();
                timer.reset();
                while (colorSensorBack.alpha() > MID_POINT_ALPHA_BACK && opModeIsActive()) {
                    waitOneFullHardwareCycle();
                    checkTimeout(timer, 3);
                }
                stopMoving();
            }


            // follow line
            i = 0;
            timer.reset();
            while (!checkTouchObject() && opModeIsActive()) {

                checkTimeout(timer, 7);

                alpha = colorSensorBack.alpha();
                //DbgLog.msg(i + " BEACON alpha " + alpha);
                //DbgLog.msg(i + " BEACON distance " + ultrasonicSensorRight.getUltrasonicLevel());

                double powerDelta = (alpha - MID_POINT_ALPHA_BACK) * (-LINE_FOLLOW_MOTOR_POWER / MID_POINT_ALPHA_BACK) * CORRECTION_MULTIPLIER;   //Delta = difference
                mecanumWheels.powerMotors(LINE_FOLLOW_MOTOR_POWER, 0, powerDelta);
                waitOneFullHardwareCycle();
                i++;
            }
            stopMoving();

            DbgLog.msg(i + " BEACON distance " + ultrasonicSensorRight.getUltrasonicLevel());
            DbgLog.msg(i + " BEACON heading " + mecanumWheels.getGyroHeading());

            //Trying several times to find the color, in case it is not initially detected.
            BeaconColor color = BeaconColor.none;
            for (int c = 0; c < 10; c++) {
                color = checkBeaconColor();
                telemetry.addData("color from beacon color: ", color);
                waitOneFullHardwareCycle();
                if (color != BeaconColor.none) {
                    break;
                }
            }

            telemetry();

            // check if we have followed the line successfully
            boolean lineFollowSuccess = false;
            //if ((color != BeaconColor.none || colorSensorBack.alpha() > MID_POINT_ALPHA_BACK/2.0) && Math.abs(mecanumWheels.getGyroHeading() - headingToBeacon) < 2.5) {
            if (color != BeaconColor.none) {
                lineFollowSuccess = true;
            }

            //color != BeaconColor.none
            if (lineFollowSuccess && opModeIsActive()) {
                dropPeople();
                waitOneFullHardwareCycle();
                if (opModeIsActive()) {
                    BeaconColor allianceColor = blueAlliance ? BeaconColor.blue : BeaconColor.red;
                    //robot has detected the beacon's color
                    //our beacon color sensor is on the right (when front of robot is facing forward)
                    if (color == allianceColor) {
                        //deploy the right button pusher
                        rightButtonPusher.setPosition(1);
                        waitOneFullHardwareCycle();
                        sleep(2500);
                        if (opModeIsActive()) {
                            rightButtonPusher.setPosition(0);
                        }
                    } else {
                        //deploy the left button pusher
                        leftButtonPusher.setPosition(1);
                        waitOneFullHardwareCycle();
                        sleep(2500);
                        if (opModeIsActive()) {
                            leftButtonPusher.setPosition(0);
                        }
                    }
                }

                telemetry();

                // move backwards to release people
                mecanumWheels.powerMotors(0.7, 0, 0);
                waitOneFullHardwareCycle();
                sleep(600);

                stopMoving();

                if (opModeIsActive()) {
                    // move sideways.
                    double strafePower = blueAlliance ? -1.0 : 1.0;
                    mecanumWheels.powerMotors(0, strafePower, 0);
                    waitOneFullHardwareCycle();
                    sleep(2500);

                    stopMoving();
                }

            }

            waitOneFullHardwareCycle();

        } else {
            stopMoving();
        }

        // Put people rod into vertical position no matter what
        peopleDrop.setPosition(0.5); // almost vertical position
        waitOneFullHardwareCycle();

        telemetry();

    }

}
