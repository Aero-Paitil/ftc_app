package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.newtonbusters.AutonomousOptions;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

/**
 * Created by Brandon Lee on 10/30/2016.
 * Updated by Athena 10/30/2016 - rotate to find an image and detect robot location
 * Updated by Athena 11/05/2016 - rotate to an angle
 * Updated by Sangmin and Jasmine 11/05/2016 - drive straight using gyro
 */

abstract class AutonomousMode extends LinearOpMode {

    enum BeaconSide {left, right, none}

    private double TILE_LENGTH = 23.5;
    private double HALF_WIDTH = 8.5;
    private double DRIVING_POWER = 0.4;
    private double MID_POINT_LIGHT_BACK = 0.3;
    //private double MAX_COUNTS_TO_WHITE = 2.26 * ENCODER_COUNTS_PER_ROTATION;

    private static final float FIELD_WIDTH = 2743.2f; //3580.0f; // millimeter
    private static final float IMAGE_HEIGHT_OVER_FLOOR = 146.05f; // millimeter

    private final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;
    private final static int SHOOT_DISTANCE1 = 11; //inches
    private final static double SHOOT_POWER1 = -0.71; //flywheel power for shooting
    private final static double SHOOT_POWER2 = -0.695; //flywheel power for shooting

    private boolean isBlue = isBlueAlliance();
    private String startTile;

    private static final String TAG = "Autonomous Mode";

    private VuforiaTrackables allImages;
    private List<VuforiaTrackable> allTrackables;

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private ColorSensor colorSensor3c;
    private ColorSensor colorSensor3a;
    private ModernRoboticsI2cColorSensor color3a, color3c;
    private OpticalDistanceSensor opticalSensor;


    private I2cController color3aController, color3cController;
    private I2cController.I2cPortReadyCallback color3aCallBack, color3cCallBack;
    private boolean colorSensorsEnabled = true;

    //defining the 4 motors
    // the motors are slaved:
    // the power that goes to the first goes to the second
    private DcMotor motorLeft1;
    //private DcMotor motorLeft2;
    private DcMotor motorRight1;
    //private DcMotor motorRight2;
    private DcMotor motorBrush;
    private DcMotor motorBelt;
    private Servo servoBeaconPad;
    private ModernRoboticsI2cGyro gyro = null;
    private DcMotor motorFlywheel;
    private Servo servoBar;

    private double[] robotLocation = null;

    abstract boolean isBlueAlliance();

    private StringBuffer out = new StringBuffer();

    int delay;
    boolean stopAfterShooting;
    String afterShootingBehavior;
    boolean firstTimeTelemetry = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //retrieve preferences from AutonomousOptions
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        String delayString = prefs.getString(AutonomousOptions.DELAY_PREF, "0 sec");
        try {
            delay = Integer.parseInt(delayString.split(" ")[0]);
        } catch (Exception e) {
            delay = 0;
        }
        startTile = prefs.getString(AutonomousOptions.START_TILE_PREF, "3rd tile");
        afterShootingBehavior = prefs.getString(AutonomousOptions.AFTER_SHOOTING_BEHAVIOR_PREF, "beacon");
        try {
            stopAfterShooting = afterShootingBehavior.equalsIgnoreCase("stop");
        } catch (Exception e) {
            stopAfterShooting = false;
        }

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro Sensor ");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        //"initializing" the motors
        motorLeft1 = hardwareMap.dcMotor.get("D1left");
        //motorLeft2 = hardwareMap.dcMotor.get("D2left");
        motorRight1 = hardwareMap.dcMotor.get("D1right");
        //motorRight2 = hardwareMap.dcMotor.get("D2right");
        motorBrush = hardwareMap.dcMotor.get("Brush");
        motorBelt = hardwareMap.dcMotor.get("Belt");
        motorFlywheel = hardwareMap.dcMotor.get("GunRight");
        // run flywheels by speed
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reset encoders
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // use running by speed mode (to make robot move straight, when equal power is applied left and right)
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range Sensor");
        colorSensor3a = hardwareMap.colorSensor.get("Color Sensor 3a");
        colorSensor3a.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor3a.enableLed(false);
        colorSensor3c = hardwareMap.colorSensor.get("Color Sensor 3c");
        colorSensor3c.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor3c.enableLed(false);
        opticalSensor = hardwareMap.opticalDistanceSensor.get("Optical");

        color3a = (ModernRoboticsI2cColorSensor) colorSensor3a;
        color3c = (ModernRoboticsI2cColorSensor) colorSensor3c;
        color3aController = color3a.getI2cController();
        color3cController = color3c.getI2cController();
        color3aCallBack = color3aController.getI2cPortReadyCallback(color3a.getPort());
        color3cCallBack = color3cController.getI2cPortReadyCallback(color3c.getPort());
        //vuforiaInit();

        gyro.resetZAxisIntegrator(); //reset gyro heading to 0

        // put the servos in correct position
        servoBeaconPad = hardwareMap.servo.get("BeaconPad");
        setPadPosition(15); // to avoid pad covering camera
        sleep(500);

        servoBar = hardwareMap.servo.get("ServoBar");
        // lower servo bar
        lowerBar();

        telemetryout("Initial: " + (isBlue ? "blue; " : "red; ") + startTile +
                "; delay: " + delay + "; after shoot: " + afterShootingBehavior);
        telemetry.update();

        //waitForStart();
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry();
            telemetry.update();
            idle();
        }

        // if we hit stop after init, nothing else should happen
        if (!opModeIsActive()) {
            return;
        }

        try {
            gyro.resetZAxisIntegrator(); //reset gyro heading to 0
            sleep(50);

            //disable color: color sensors are only enabled when detecting color
            color3aController.deregisterForPortReadyCallback(color3a.getPort());
            color3cController.deregisterForPortReadyCallback(color3c.getPort());
            colorSensorsEnabled = false;

            // raise servoBar
            liftBar();

            if (delay > 0) {
                sleep(1000 * delay);
            }

            if (startTile.startsWith("3rd")) {
                sequenceFromTile3();
            } else {
                sequenceFromTile4();
            }

        } finally {
            try {
                File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/lastrun.txt");
                telemetry.addData("Wrote File", file.getAbsolutePath());

                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

                String timestamp = new SimpleDateFormat("MMMdd_HHmm").format(new Date());
                file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/lastrun_" + timestamp + ".txt");
                telemetry.addData("File", file.getAbsolutePath());

                outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

            } catch (Exception e) {
                telemetry.addData("Exception", "File write failed: " + e.toString());
            }
        }
    }

    /**
     * This is autonomous command sequence from the 3rd tile
     * from the corner of the Vortex.
     *
     * @throws InterruptedException
     */
    private void sequenceFromTile3() throws InterruptedException {
        // starting with robot at the wall in the middle of the third tile
        // robot facing backward

        int angle, fromAngle;

        //start shooting at the position1 (3rd tile from the corner)
        moveOnForShooting(0.4, SHOOT_DISTANCE1, SHOOT_POWER1);
        motorBelt.setPower(1);
        sleep(2500);
        motorBelt.setPower(0);
        powerFlywheels(false);

        telemetryout("After shooting");

        if (stopAfterShooting) {
            moveByInches(11);
            powerMotors(0, 0);
            showTelemetry();
            return;
        }
        if (afterShootingBehavior.equals("rampPark")){
            parkOnRamp();
            return;
        }
        // blue: rotate 46 from heading 0
        // red: rotate -46 from heading 0
        angle = isBlue ? 43 : -47;
        rotate(angle, 0);

        telemetryout("After rotated " + angle + " degrees");
        // make sure the robot has settled to get correct heading
        sleep(200);

        // drive almost to the white line
        boolean movedRequiredDistance = moveByInchesGyro(-0.92, angle, 41, -0.3);
        telemetryout("Moved 41  inches " + movedRequiredDistance);

        if (!movedRequiredDistance) {
            // if we didn't travel the correct distance,
            // we probably hit something
            powerMotors(0, 0);
            showTelemetry();
            return;
        }


        // go to white line (first beacon)
        boolean foundWhiteLine = driveUntilWhite(-0.3, angle, 20);
        telemetryout("Found white line: " + foundWhiteLine);
        if (!foundWhiteLine) {
            // if line is not detected stop and
            // show telemetry while op mode is active
            powerMotors(0, 0);
            showTelemetry();
            return;
        }

        // go 8 inches past white line
        double pastLineInches = -8; //
        moveByInches(pastLineInches, 0.3);

        telemetryout("Passed " + pastLineInches + " inches after white line");
        // blue: rotate ~30 degrees CW from heading 45
        // red: rotate ~30 degrees CCW from heading -45
        fromAngle = angle;
        angle = isBlue ? 30 : -30;
        rotate(angle, fromAngle, 0.3);

        lowerBar();

        telemetryout("Rotated to white line");
        followLine(-0.3, MID_POINT_LIGHT_BACK);

        telemetryout("After followed line");
        // detect color
        BeaconSide beaconSide = detectColor(true);

        if (beaconSide != BeaconSide.none) {

            // if color is detected, move forward to hit the beacon
            sleep(500); //Give time for pad to get  into position
            boolean atWall = driveUntilHit(5, -0.3);
            if (atWall) {
                telemetryout("At the wall");
                // shimmy on the beacon side
                // shimmy(beaconSide);
                moreShimmyIfNeeded();
                telemetryout("After shimmy");
            }
        }

        //TODO: write short distance gyro drive with large coefficient
        // move 8 inches back
        moveByInches(8);

        telemetryout("Moved back 8 inches");
        // blue: rotate -90 degrees from heading 90
        // red: rotate 90 degrees from heading -90
        fromAngle = isBlue ? 90 : -90;
        angle = isBlue ? -90 : 90;
        rotate(angle, fromAngle);

        telemetryout("Rotated along the wall");

        liftBar();

        sleep(200);

        // drive the distance between two white lines
        movedRequiredDistance = moveByInchesGyro(-0.92, 0, 33, -0.3);
        telemetryout("Moved required distance 2: " + movedRequiredDistance);
        if (!movedRequiredDistance) {
            // if we didn't travel the correct distance,
            // we probably hit something
            powerMotors(0, 0);
            showTelemetry();
            return;
        }

        // go to white line (second beacon)
        foundWhiteLine = driveUntilWhite(-0.3, 0, 20);
        telemetryout("Found white line 2: " + foundWhiteLine);
        if (!foundWhiteLine) {
            // if line is not detected stop and
            // show telemetry while op mode is active
            powerMotors(0, 0);
            lowerBar();
            showTelemetry();
            return;
        } else {
            // go 8 inches past white line
            pastLineInches = -8 - 5; // extra 5 inches to clear space for the turn
            moveByInches(pastLineInches);
            //moveByInchesGyro(-0.3, 0, Math.abs(pastLineInches), -0.3 );
            lowerBar();
            sleep(100);
            moveByInches(5);
        }

        telemetryout("Passed " + pastLineInches + " inches after white line 2");

        // blue: rotate 90 degrees CW from heading 0
        // red: rotate 90 degrees CCW from heading 0
        fromAngle = 0;
        angle = isBlue ? 65 : -65;
        rotate(angle, fromAngle);

        telemetryout("Rotated to white line 2");

        followLine(-0.3, MID_POINT_LIGHT_BACK);

        telemetryout("Followed line 2");

        // detect color
        beaconSide = detectColor(true);

        if (beaconSide != BeaconSide.none) {

            // if color is detected, move forward to hit the beacon
            sleep(500); //Give time for pad to get  into position
            boolean atWall = driveUntilHit(5, -0.3);

            if (atWall) {
                telemetryout("At the wall 2");
                // shimmy on the beacon side
                //shimmy(beaconSide);
                moreShimmyIfNeeded();
                telemetryout("After shimmy 2");
            }
        }

        motorBrush.setPower(-1);

        //first travel 1/10th of a circle with the radius of the tile length
        //red - clockwise; blue - counterclockwise
        moveByArch(TILE_LENGTH, 1/10.0, !isBlue, 1.0);

        angle = isBlue ? 45 : -45;
        moveByInchesGyro(0.92, angle, 43, 0.92);

        powerMotors(0, 0);
        motorBrush.setPower(0);

        telemetryout("End");
    }


    /**
     * This is autonomous command sequence from the 4th tile
     * from the corner of the Vortex.
     *
     * @throws InterruptedException
     */
    private void sequenceFromTile4() throws InterruptedException {
        int angle, fromAngle;

        // move to the middle of the next tile
        double distance = Math.abs(-TILE_LENGTH - (TILE_LENGTH - 18) / 2);
        moveOnForShooting(0.4, distance, SHOOT_POWER2);

        // to make a rotation of 45 degrees to shoot the ball from heading zero.
        // blue: rotate 45 degrees CW from heading 0
        // red: rotate -45 degrees CCW from heading 0
        angle = isBlue ? 45 : -45;
        rotate(angle, 0);

        // shooting the ball(s)
        motorBelt.setPower(1);
        sleep(4000);
        motorBelt.setPower(0);
        powerFlywheels(false);

        if (stopAfterShooting) {
            powerMotors(0, 0);
            showTelemetry();
            return;
        }


        if (afterShootingBehavior.equals("ballPark")) {
            moveBallAndPark();
            return;
        } else if (afterShootingBehavior.equals("rampPark")){
            parkOnRamp();
            return;
        }

        // blue: rotate 90 degrees CW from heading 45
        // red: rotate -90 degrees CCW from heading -45
        fromAngle = angle;
        angle = isBlue ? 43 : -43;
        rotate(angle, fromAngle);

        // Go backwards a quarter of a circle
        moveByArch(2*TILE_LENGTH, 1/4.0, !isBlue, -0.6);
        powerMotors(0, 0);

        // blue: rotate 30 degrees CW from heading 0
        // red: rotate -26 degrees CCW from heading 0 (red does not get as close)
        angle = isBlue ? 30 : -26;
        rotate(angle, 0);

        //Move 18 inches
        moveByInches(18, -0.5);

        // go to white line
        boolean foundWhiteLine = driveUntilWhite(-0.2, angle, 32);
        telemetryout("Found white line: " + foundWhiteLine);
        if (!foundWhiteLine) {
            // if line is not detected stop and
            // show telemetry while op mode is active
            powerMotors(0, 0);
            showTelemetry();
            return;
        }

        // go 8 inches past white line
        double pastLineInches = -8; //
        moveByInches(pastLineInches);

        telemetryout("Passed " + pastLineInches + " inches after white line");

        // blue: rotate 35 degrees CW from heading 45
        // red: rotate 35 degrees CCW from heading -45
        fromAngle = angle;
        angle = isBlue ? 35 : -35;
        rotate(angle, fromAngle);

        telemetryout("Rotated to white line");

        lowerBar();

        BeaconSide beaconSideFar = followLine(-0.3, MID_POINT_LIGHT_BACK);

        telemetryout("After followed line");

        // detect color
        BeaconSide beaconSide = detectColor(true);

        telemetryout("Color detected: " + beaconSide);

        if (beaconSide == BeaconSide.none && beaconSideFar == BeaconSide.none) {
            // if color is not detected stop and
            // show telemetry while op mode is active
            powerMotors(0, 0);
            showTelemetry();
            return;
        }

        // if color is detected, move forward to hit the beacon
        sleep(500); //Give time for pad to get  into position
        boolean atWall = driveUntilHit(5, -0.3);

        if (atWall){
            telemetryout("At the wall");
            // shimmy on the beacon side
            shimmy(beaconSide);
            moreShimmyIfNeeded();
            telemetryout("After shimmy");
        }


        // move away from beacon
        moveByInches(8);

        telemetryout("Moved back 8 inches");

        powerMotors(0, 0);


    }

    private void parkOnRamp() throws InterruptedException {
        int fromAngle, angle, distance;
        if (startTile.startsWith("3rd")) {
            fromAngle = 0;
            angle = isBlue ? -75 : 75;
            distance = isBlue ? 45 : 50;
        } else {
            fromAngle = isBlue ? 45 : -45;
            angle = isBlue ? -110 : 117;
            distance = isBlue ? 70 : 75;
        }
        rotate(angle, fromAngle);
        lowerBar();
        moveByInches(distance, 0.5);
    }

    private void moveBallAndPark() throws InterruptedException {
        sleep((10 - delay) * 1000);
        int fromAngle = isBlue ? 45 : -45;
        int angle = isBlue ? 170 : -170;
        rotate(angle, fromAngle);
        motorBrush.setPower(-1);
        moveByInches(1.75 * TILE_LENGTH);
    }
    private void liftBar(){
        servoBar.setPosition(95.0 / 255);
    }
    private void lowerBar(){
        servoBar.setPosition(215.0 / 255);
    }


    //Tiny rotation clockwise or counter clockwise
    private void smallrotate(String direction) throws InterruptedException {
        telemetryout("before small rotate " + direction);
        if (direction.equals("clockwise")) {
            powerMotors(0.3, -0.3);
        } else {
            powerMotors(-0.3, 0.3);
        }
        sleep(150);
        powerMotors(0, 0);
        sleep(150);
    }


    private void shimmy(BeaconSide beaconSide) throws InterruptedException {
        if (beaconSide == BeaconSide.left) {
            powerMotors(0.3, -0.3);
        } else {
            powerMotors(-0.3, 0.3);
        }
        sleep(150);
        powerMotors(0, 0);
        sleep(150);
    }

    private void moreShimmyIfNeeded() throws InterruptedException {
        BeaconSide beaconSide;
        while (opModeIsActive()) {
            // check color, if detected, keep shimmying
            beaconSide = detectColor();
            telemetryout("Color detected: " + beaconSide);
            if (beaconSide == BeaconSide.none) {
                return;
            }
            // shimmy on the beacon side for the second time
            shimmy(beaconSide);
        }
    }

    private BeaconSide detectColor() throws InterruptedException {
        return detectColor(false);
    }

    private BeaconSide detectColor(boolean handleUnknown) throws InterruptedException {
        //enable color
        color3aController.registerForI2cPortReadyCallback(color3aCallBack, color3a.getPort());
        color3cController.registerForI2cPortReadyCallback(color3cCallBack, color3c.getPort());
        colorSensorsEnabled = true;
        sleep(150);

        int padPosition = 0;
        // detect color (red alliance
        BeaconSide beaconSide = BeaconSide.none;
        boolean colorDetected = false;
        boolean maybe = true;
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (opModeIsActive() && maybe) {
            maybe = false;
            if (colorSensor3a.red() > colorSensor3a.blue() && colorSensor3c.blue() > colorSensor3c.red()) {
                // 3a is more red, 3c is more blue
                colorDetected = true;
                padPosition = isBlue ? 240 : 15;
                setPadPosition(padPosition);
            } else if (colorSensor3c.red() > colorSensor3c.blue() && colorSensor3a.blue() > colorSensor3a.red()) {
                // 3c is more red, 3a is more blue
                colorDetected = true;
                padPosition = isBlue ? 15 : 240;
                setPadPosition(padPosition);
            } else if (handleUnknown) {
                if ((colorSensor3a.red() > colorSensor3a.blue() && colorSensor3c.blue() == colorSensor3c.red()) ||
                        (colorSensor3c.red() == colorSensor3c.blue() && colorSensor3a.blue() > colorSensor3a.red())) {
                    // rotate right
                    smallrotate("clockwise");
                    maybe = true;
                } else if ((colorSensor3c.red() > colorSensor3c.blue() && colorSensor3a.blue() == colorSensor3a.red()) ||
                        (colorSensor3a.red() == colorSensor3a.blue() && colorSensor3c.blue() > colorSensor3c.red())) {
                    //rotate left
                    smallrotate("counterclockwise");
                    maybe = true;
                }
                if (Math.abs(Math.abs(getGyroRawHeading()) - 90) > 10) {
                    telemetryout("gyro tolerance exceeded");
                    maybe = false;
                }
            }
            telemetry();
            idle();
        }
        if (colorDetected) {
            beaconSide = padPosition == 15 ? BeaconSide.left : BeaconSide.right;
        }
        telemetryout("Color detected: " + beaconSide);

        // disable color
        color3aController.deregisterForPortReadyCallback(color3a.getPort());
        color3cController.deregisterForPortReadyCallback(color3c.getPort());
        colorSensorsEnabled = false;

        return beaconSide;
    }

    private boolean driveUntilHit(int distincm, double drivingPower) throws InterruptedException {
        powerMotors(drivingPower, drivingPower);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //time out if robot cannot approach wall
        while (rangeSensor.getDistance(DistanceUnit.CM) > distincm) {
            idle();
            if (timer.milliseconds() > 4000) {
                powerMotors(0,0);
                return false;
            }
        }
        powerMotors(0, 0);
        sleep(600);
        return true;
    }

    private void setPadPosition(int pos) {
        servoBeaconPad.setPosition(pos / 255.0);
    } // from 0 to 255

    private void showTelemetry() throws InterruptedException {
        while (opModeIsActive()) {
            telemetry();
            idle();
        }
    }

    public void telemetry() {
        //telemetry.addData("Count: ", motorRight1.getCurrentPosition());
        telemetry.addData("Alliance", isBlue ? "blue" : "red");
        telemetry.addData(startTile, "delay " + delay + "; after shoot: " + afterShootingBehavior);
        telemetry.addData("Color 3c - R/G/B ", colorSensor3c.red() + "/" + colorSensor3c.green() + "/" +
                colorSensor3c.blue());
        telemetry.addData("Color 3a - R/G/B ", colorSensor3a.red() + "/" + colorSensor3a.green() + "/" +
                colorSensor3a.blue());
        telemetry.addData("Optical Light ", "%.4f", opticalSensor.getLightDetected());
        telemetry.addData("Gyro Reading ", getGyroRawHeading());
        telemetry.addData("Distance ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        if (robotLocation != null && robotLocation.length == 3) {
            telemetry.addData("Location", "x = %.0f, y = %.0f, z = %.0f", robotLocation[0], robotLocation[1], robotLocation[2]);
        }
        telemetry.update();
    }

    private void telemetryout(String step) {
       if (firstTimeTelemetry) {
            out.append("Time,Step,Optical Light,Gyro Reading,Distance,Wheel Encoder Position,Voltage,Color3c - R/G/B,Color3a - R/G/B").append("\n");
            firstTimeTelemetry = false;
        }
        out.append("\n").append(new SimpleDateFormat("MMMdd_HHmm:ss").format(new Date())).append(",");
        out.append(step).append(",");
        out.append(opticalSensor.getLightDetected()).append(",");
        out.append(getGyroRawHeading()).append(",");
        out.append(rangeSensor.getDistance(DistanceUnit.CM)).append("cm,");
        out.append(motorLeft1.getCurrentPosition()).append(",");
        out.append(this.hardwareMap.voltageSensor.iterator().next().getVoltage()).append(",");
        if (colorSensorsEnabled) {
            out.append(colorSensor3c.red()).append("/").append(colorSensor3c.green()).append("/").append(colorSensor3c.blue()).append(",");
            out.append(colorSensor3a.red()).append("/").append(colorSensor3a.green()).append("/").append(colorSensor3a.blue()).append(",");
        } else {
            out.append(",,");
        }
    }


    private void powerMotors(double leftForward, double rightForward) throws InterruptedException {
        // the left side direction is reversed
        motorLeft1.setPower(-leftForward);
        motorRight1.setPower(rightForward);
        idle();
    }

    private void powerFlywheels(boolean doPower) {
        // the flywheels should move out in the opposite direction
        if (doPower) {
            motorFlywheel.setPower(-0.5);
            sleep(1200);
            motorFlywheel.setPower(-0.72);
            sleep(1200);
        } else {
            motorFlywheel.setPower(0);
        }
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        motorLeft1.setMode(runMode);
        motorRight1.setMode(runMode);
    }

    private void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        motorLeft1.setZeroPowerBehavior(behavior);
        motorRight1.setZeroPowerBehavior(behavior);
    }

    private void vuforiaInit() {
        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            SharedPreferences prefs = getSharedPrefs(hardwareMap);

            parameters.vuforiaLicenseKey = prefs.getString("vuforiaLicenseKey", null);
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            allImages = vuforia.loadTrackablesFromAsset("FTC_2016-17");
            allTrackables = new ArrayList<>(allImages);

            VuforiaTrackable wheelTarget = allImages.get(0);
            wheelTarget.setName("WheelTarget");
            VuforiaTrackable toolsTarget = allImages.get(1);
            toolsTarget.setName("ToolsTarget");
            VuforiaTrackable legosTarget = allImages.get(2);
            legosTarget.setName("LegosTarget");
            VuforiaTrackable gearsTarget = allImages.get(3);
            gearsTarget.setName("GearsTarget");

            OpenGLMatrix wheelTargetLocationOnField = OpenGLMatrix
                    .translation(FIELD_WIDTH / 12, FIELD_WIDTH / 2, IMAGE_HEIGHT_OVER_FLOOR)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            wheelTarget.setLocation(wheelTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(wheelTargetLocationOnField), wheelTarget.getName());

            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELD_WIDTH / 4, FIELD_WIDTH / 2, IMAGE_HEIGHT_OVER_FLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            legosTarget.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(legosTargetLocationOnField), legosTarget.getName());

            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELD_WIDTH / 2, FIELD_WIDTH / 4, IMAGE_HEIGHT_OVER_FLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            toolsTarget.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(toolsTargetLocationOnField), toolsTarget.getName());

            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELD_WIDTH / 2, -FIELD_WIDTH / 12, IMAGE_HEIGHT_OVER_FLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            gearsTarget.setLocation(gearsTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(gearsTargetLocationOnField), gearsTarget.getName());

            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(0, 0, 0);
//                        .translation(209.55f, 177.8f, 266.7f) // these are all in mm
//                        .multiplied(Orientation.getRotationMatrix(
//                                AxesReference.EXTRINSIC, AxesOrder.YZY,
//                                AngleUnit.DEGREES, 90, -90, 0));
            RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));


            ((VuforiaTrackableDefaultListener) wheelTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) legosTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) toolsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) gearsTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();

        } catch (Exception e) {
            RobotLog.ii(TAG, "Exception=%s", e.getMessage());
            System.out.print(e.getMessage());
        }
    }

    /**
     * returns three numbers: x, y, and z (angle) as an array or null if the location of the robot is unknown.
     */
    private void updateRobotLocation() {
        boolean isVisible;
        VuforiaTrackableDefaultListener listener;
        try {
            for (VuforiaTrackable trackable : allTrackables) {
                listener = ((VuforiaTrackableDefaultListener) trackable.getListener());
                isVisible = listener.isVisible();
                //telemetry.addData(trackable.getName(), isVisible ? "Visible" : "Not Visible");
                if (!isVisible) {
                    continue;
                }

                OpenGLMatrix lastLocation = listener.getUpdatedRobotLocation();
                if (lastLocation != null) {

                    RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                    //telemetry.addData("Pos", format(lastLocation));

                    VectorF v = lastLocation.getTranslation();
                    double x = v.get(0);
                    double y = v.get(1);

                    Orientation ori = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double zAngle = ori.thirdAngle;

                    robotLocation = new double[]{x, y, zAngle};
                }
            }
            //telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            System.out.println(e.getMessage());
        }
    }

    private String format(OpenGLMatrix transformationMatrix) {
        //VectorF v = transformationMatrix.getTranslation();
        //return ("{" + v.get(0) + ", " + v.get(1) + ", " + v.get(2) + "}");
        return transformationMatrix.formatAsTransform();
    }

    private static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }

    private void moveByInches(double inches) throws InterruptedException {
        moveByInches(inches, DRIVING_POWER);
    }

    private void moveByInches(double inches, double drivingPower) throws InterruptedException { // moves 26.5 in one rotation
        telemetry();
        int counts = motorRight1.getCurrentPosition();
        double sign = Math.round(inches / Math.abs(inches));
        powerMotors(sign * drivingPower, sign * drivingPower);
        while (opModeIsActive() && Math.abs(motorRight1.getCurrentPosition() - counts) < inchesToCounts(inches)) {
            idle();
            telemetry.addData("Raw heading", getGyroRawHeading());
            telemetry.update();
        }
        powerMotors(0, 0);
    }

    private void rotate(int angle, int fromRawHeading) throws InterruptedException {
        rotate(angle, fromRawHeading, DRIVING_POWER);
    }

    // pcircle - part of the circle, positive - clockwise
    private void rotate(int angle, int fromRawHeading, double power) throws InterruptedException {
        sleep(100); // to make sure robot stopped moving

        int counts = motorRight1.getCurrentPosition();

        // make an adjustment for the error in current gyro heading
        double pcircleError = getRawHeadingError(fromRawHeading) / 360.0;
        double pcircle = angle / 360.0;
        pcircle = pcircle - pcircleError;
        telemetry.addData("pcircle", pcircle * 360);
        telemetry.update();

        double sign = Math.round(pcircle / Math.abs(pcircle));
        powerMotors(sign * power, -sign * power);
        // assuming 16 inches between wheels, 8 inches radius - 50.24 in
        // assuming 15 inches between wheels, 7.5 inches radius - 46.24 in
        // assuming 15.5 inches between wheels, 7.75 inches radius - 48.67 in
        while (Math.abs(motorRight1.getCurrentPosition() - counts) < Math.abs(pcircle) * inchesToCounts(48.67)) {
            idle();
        }
        powerMotors(0, 0);
    }

    private double inchesToCounts(double inches){
        return Math.abs(ENCODER_COUNTS_PER_ROTATION * inches / 26.5);
    }

    /**
     * Drive until white line
     *
     * @param drivingPower        power (positive - forward, negative - backward)
     * @param headingToBeaconZone raw gyro heading
     * @param maxInches           - maximum inches to travel
     * @return true if traveled distance, otherwise false
     * @throws InterruptedException
     */
    private boolean moveByInchesGyro(double drivingPower, int headingToBeaconZone, double maxInches, double nextPower) throws InterruptedException {

        int initialcount = motorLeft1.getCurrentPosition();
        double error, clockwiseSpeed;
        double kp = 0.03; //experimental coefficient for proportional correction of the direction
        double countsSinceStart = Math.abs(motorLeft1.getCurrentPosition() - initialcount);
        double slope = (nextPower - drivingPower) / Math.min(10, maxInches); // this slope is for calculating power
        double countsForGradient = inchesToCounts(maxInches > 10 ? maxInches - 10 : 0);
        double motorPower = drivingPower;
        while (opModeIsActive() && countsSinceStart < inchesToCounts(maxInches)) {
            // error CCW - negative, CW - positive
            error = getRawHeadingError(headingToBeaconZone);
            //telemetryout("moveByInchesGyro function => error is: " + error);

            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else if (Math.abs(error) >= 1 && Math.abs(error) <= 20) {
                clockwiseSpeed = kp * error / 4;
            } else {
                clockwiseSpeed = kp * Math.abs(error) / error;
            }
            //telemetryout("moveByInchesGyro function => clockwiseSpeed = " + clockwiseSpeed);
            //telemetryout("moveByInchesGyro function => distance = " +distance);
            //clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            telemetry.addData("Error", error);
            telemetry.update();


            if (countsSinceStart > countsForGradient){
                motorPower = slope * (countsSinceStart-inchesToCounts(maxInches)) + nextPower;
            }
            powerMotors(Range.clip(motorPower - clockwiseSpeed, -1.0, 1.0), Range.clip(motorPower + clockwiseSpeed, -1.0, 1.0));

            countsSinceStart = Math.abs(motorLeft1.getCurrentPosition() - initialcount);
            //telemetryout("moveByInchesGyro function => powerMotors = "+ (drivingPower-clockwiseSpeed) + "and " + (drivingPower + clockwiseSpeed));
        }
        return opModeIsActive();
    }

    private boolean driveUntilWhite(double drivingPower, int headingToBeaconZone) throws InterruptedException {
        return driveUntilWhite(drivingPower, headingToBeaconZone, Integer.MAX_VALUE);
    }

    /**
     * Drives until white line, does not stop
     *
     * @param drivingPower        driving power
     * @param headingToBeaconZone raw gyro heading
     * @param maxInches           maximum distance
     * @return true is white is detected, false otherwise
     * @throws InterruptedException
     */
    private boolean driveUntilWhite(double drivingPower, int headingToBeaconZone, double maxInches) throws InterruptedException {


        int initialcount = motorLeft1.getCurrentPosition();
        double error, clockwiseSpeed;
        double kp = 0.03; //experimental coefficient for proportional correction of the direction
        //alpha() is to measure the brightness.
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        //double alpha = colorSensorBottom.alpha();
        double light = opticalSensor.getLightDetected();
        while (opModeIsActive() && light < MID_POINT_LIGHT_BACK && Math.abs(motorLeft1.getCurrentPosition() - initialcount) < inchesToCounts(maxInches)) {
            // error CCW - negative, CW - positive
            error = getRawHeadingError(headingToBeaconZone);
            //don't do any correction
            //if heading error < 1 degree
            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else if (Math.abs(error) >= 1 && Math.abs(error) <= 20) {
                clockwiseSpeed = kp * error / 4;
            } else {
                clockwiseSpeed = kp * Math.abs(error) / error;
            }
            //clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            telemetry.addData("Error", error);
            telemetry.update();
            //DbgLog.msg(i + " clockwise speed "+clockwiseSpeed);

            powerMotors(drivingPower - clockwiseSpeed, drivingPower + clockwiseSpeed);
            light = opticalSensor.getLightDetected();
        }
        //motorBrush.setPower(0);
        return light >= MID_POINT_LIGHT_BACK;
    }

    /*
     * Returns beacon side at which the alliance color was detected (before line follow)
     */
    private BeaconSide followLine(double drivingPower, double targetWhiteValue) throws InterruptedException {

        double error, clockwiseSpeed;
        double kp = 0.065; //experimental coefficient for proportional correction of the direction
        double light = opticalSensor.getLightDetected();

        if (opModeIsActive() && light < targetWhiteValue) {
            if (isBlue) {
                powerMotors(0.15, -0.15);
            } else {
                powerMotors(-0.15, 0.15);
            }
            light = opticalSensor.getLightDetected();
            while (opModeIsActive() && light < targetWhiteValue) {
                idle();
                light = opticalSensor.getLightDetected();
            }
            powerMotors(0, 0);
        }

        telemetryout("Stopped at line");
        BeaconSide beaconSide = detectColor();

        // move back to the edge
        if (opModeIsActive()) {
            if (isBlue) {
                powerMotors(-0.12, 0.12);
            } else {
                powerMotors(0.12, -0.12);
            }
            sleep(25); // let it move a bit back
            light = opticalSensor.getLightDetected();
            while (opModeIsActive() && light < targetWhiteValue) {
                idle();
                light = opticalSensor.getLightDetected();
            }
            powerMotors(0, 0);
            telemetryout("Moved back a bit");
        }

        double distance = rangeSensor.getDistance(DistanceUnit.CM);
        I2cController gyroController = gyro.getI2cController();
        I2cController.I2cPortReadyCallback gyroCallBack = gyroController.getI2cPortReadyCallback(gyro.getPort());
        gyroController.deregisterForPortReadyCallback(gyro.getPort());

        while (opModeIsActive() && distance > 11) {

            light = opticalSensor.getLightDetected();

            error = light - targetWhiteValue;
            clockwiseSpeed = kp * error / 0.5;

            if (!isBlue) {
                clockwiseSpeed = -clockwiseSpeed;
            }
            out.append(light).append(",").append(error).append("\n");

            //clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            telemetry.addData("Error", error);
            telemetry.update();

            powerMotors(drivingPower - clockwiseSpeed, drivingPower + clockwiseSpeed);
            sleep(5);
            distance = rangeSensor.getDistance(DistanceUnit.CM);
        }
        powerMotors(0, 0);

        //enable sensors again
        gyroController.registerForI2cPortReadyCallback(gyroCallBack, gyro.getPort());
        sleep(150);

        return beaconSide;
    }

    //getIntegratedZValue is positive when moving ccw. We want it to behave the same way as getGyroHeading, so we changed the sign.
    private int getGyroRawHeading() {
        //int heading = -gyro.getIntegratedZValue();
        //telemetry.addData("Raw heading", heading);
        //return heading;
        return -gyro.getIntegratedZValue();
    }

    //cc error is positive, ccw error is negative
    private double getRawHeadingError(double requiredRawHeading) {
        double diffInHeading = getGyroRawHeading() - requiredRawHeading;
        //telemetryout("Inside getRawHeadingError method: diffInHeading = " + diffInHeading);
        return diffInHeading;
    }

    private void moveOnForShooting(double power, double distance, double shootPower) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        motorFlywheel.setPower(-0.5);
        timer.reset();

        this.powerMotors(-power, -power);

        int startPosition = motorLeft1.getCurrentPosition();
        int currentPosition = startPosition;

        while (opModeIsActive() && Math.abs(currentPosition - startPosition) < inchesToCounts(distance) &&
                timer.milliseconds()<1200) {
            currentPosition = motorLeft1.getCurrentPosition();
            idle();
        }
        motorFlywheel.setPower(shootPower);
        while (opModeIsActive() && (Math.abs(currentPosition - startPosition) < inchesToCounts(distance))){
            currentPosition = motorLeft1.getCurrentPosition();
            idle();
        }
        powerMotors(0, 0);
        int currentms = (int)timer.milliseconds();
        if (currentms < 2400){
            sleep(2400-currentms);
        }
    }

    /**
     * method for moving robot part of circle
     * @param radius - radius of circle
     * @param pCircle - fraction of circle
     * @param clockwise - moving clockwise?
     * @param power - power to outer wheel
     * @throws InterruptedException
     */
    private void moveByArch(double radius, double pCircle, boolean clockwise, double power) throws InterruptedException {
        double outerWheelDist = 2 * Math.PI * (radius + HALF_WIDTH) * pCircle;
        double powerRatio = (radius - HALF_WIDTH)/(radius + HALF_WIDTH);
        DcMotor outerMotor;
        int counts;
        if (!clockwise) {
            outerMotor = motorRight1;
            powerMotors(powerRatio, power);
        } else {
            outerMotor = motorLeft1;
            powerMotors(power, powerRatio);
        }

        counts = outerMotor.getCurrentPosition();

        while (opModeIsActive() && Math.abs(outerMotor.getCurrentPosition() - counts) < inchesToCounts(outerWheelDist)) {
            idle();
            telemetry.addData("Raw heading", getGyroRawHeading());
            telemetry.update();
        }
    }

    /*private double filterRangeSensorData(double curDistance, double preDistance,ArrayList<Double> rangeArray){
        double rangeAvg = 0;
        double lastRangeAvg = 0.0;
        double sum = 0;
        double lastSum = 0;

        if((curDistance!=preDistance ) && curDistance < 130) {
            rangeArray.add(curDistance);
            for (double distance : rangeArray) {
                sum += distance;
            }
            rangeAvg = sum / rangeArray.size();

            //retrieve last 10 readings, if avg below 13, the robot is stuck with and object, return distance = 0 to stop the robot
            telemetryout(" curDistance=" + curDistance + " preDist=" + preDistance + " sum=" + sum + " size=" + rangeArray.size() + " rArray=" + rangeArray + " rAvg=" + rangeAvg);

            //filter exceptional readings from rangeSensor,  && Math.abs(curDistance-rangeAvg)> 15
            if(curDistance <= 13 ){
                if(rangeArray.size() >=20) {
                    List<Double> last20 = rangeArray.subList(rangeArray.size() - 20, rangeArray.size());
                    for (double reading : last20) {
                        lastSum += reading;
                    }
                    lastRangeAvg = lastSum / 20;
                    telemetryout("lastRangeAvg = " + lastRangeAvg + " return=>"+ (lastRangeAvg > 10 ? preDistance : 0));
                    curDistance = lastRangeAvg > 10 ? preDistance : 0;
                }else{
                    telemetryout("obect is close to robot");
                    curDistance = preDistance;
                }
            }
        }
        telemetryout("before exit, curDistance = "+ curDistance);
        return curDistance;
    }*/


}
