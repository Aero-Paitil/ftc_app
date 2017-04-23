package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.newtonbusters.AutonomousOptions;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Created by Brandon Lee on 10/30/2016.
 * Updated by Athena 10/30/2016 - rotate to find an image and detect robot location
 * Updated by Athena 11/05/2016 - rotate to an angle
 * Updated by Sangmin and Jasmine 11/05/2016 - drive straight using gyro
 */

abstract class AutonomousMode extends LinearOpMode {

    boolean TESTING = false;

    void testSequence() throws InterruptedException {}

    // subclasses for red and blue should implement this method
    abstract boolean isBlueAlliance();
    
    enum BeaconSide {left, right, none}

    final double TILE_LENGTH = 23.5; //Length of one tile in inches
    final double HALF_WIDTH = 7.6; //Half of the distance between the wheels in inches
    final double DRIVING_POWER = 0.4; //The default driving power
    final double MID_POINT_LIGHT_BACK = 0.2; //Optical distance sensor reading on the edge of the white line

    private final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140; //Encoder counts per rotation
    private final static int SHOOT_DISTANCE1 = 11; //The distance moved before the robot shoots //inches was 11
    private final static double SHOOT_POWER1 = -0.645; //-0.72; //flywheel power for shooting
    private final static double SHOOT_POWER2 = -0.632; //-0.695; //flywheel power for shooting

    private boolean isBlue = isBlueAlliance(); //Determines if the robot is running blue or red
    private String startTile; //Tile the robot starts on before it moves

    //Sensor Variables
     ModernRoboticsI2cGyro gyro = null;
     ModernRoboticsI2cRangeSensor rangeSensor;
     ModernRoboticsI2cColorSensor colorSensor3a, colorSensor3c;
     OpticalDistanceSensor opticalSensor;

    //Controllers to enable and disable the colour sensors
    //The update rate of the sensors slows with every new  sensor added,
    //so this allows for a quicker update rate when needed.
     private I2cController color3aController, color3cController;
     private I2cController.I2cPortReadyCallback color3aCallBack, color3cCallBack;
     private boolean colorSensorsEnabled = true;

    //defining the 4 motors
    // the drive motors are slaved:
    // the power that goes to the first goes to the second
     DcMotor motorLeft1; //motorLeft2 is ganged
     DcMotor motorRight1; //motorRight2 is ganged
     DcMotor motorBrush; //intake brush
     DcMotor motorBelt; //intake belt
     DcMotor motorFlywheel; //flywheel shooter

     Servo servoBeaconPad; // beacon pushing pad
     Servo servoBar; // the bar to push big ball away

    // flippers on the bottom of the robot to kick small balls away
    // if not removed, small balls can get stuck between the wall and the wheels of the robot
     Servo kickServo2; //First flipper on the bottom of the robot
     Servo kickServo3; //Second flipper on the bottom of the robot

     StringBuffer out = new StringBuffer(); //String Buffer

    private int delay; //option: delay seconds before autonomous sequence starts
    private boolean stopAfterShooting; //option: should we stop after shooting
    private String afterShootingBehavior; //option: what to do after shooting to do after the shooting

    @Override
    public void runOpMode() throws InterruptedException {
        //retrieve preferences from AutonomousOptions
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);

        //delays if the option is set for it to do so [delay is in seconds]
        String delayString = prefs.getString(AutonomousOptions.DELAY_PREF, "0 sec");
        try {
            delay = Integer.parseInt(delayString.split(" ")[0]);
        } catch (Exception e) {
            delay = 0;
        }
        //Two start positions ==> on third or fourth tile, controlled by Autonomous Options
        startTile = prefs.getString(AutonomousOptions.START_TILE_PREF, "3rd tile");
        afterShootingBehavior = prefs.getString(AutonomousOptions.AFTER_SHOOTING_BEHAVIOR_PREF, "beacon");
        try {
            stopAfterShooting = afterShootingBehavior.equalsIgnoreCase("stop");
        } catch (Exception e) {
            stopAfterShooting = false;
        }
        if (startTile.startsWith("4th") && afterShootingBehavior != null && afterShootingBehavior.equals("beacon")) {
            telemetry.addData("ERROR" , "beacon is not supported from 4th tile");
            telemetry.update();
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

        // driving in speed mode (to make robot move straight, when equal power is applied left and right)
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range Sensor");
        colorSensor3a = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("Color Sensor 3a");
        colorSensor3a.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor3a.enableLed(false); //Put the colour sensor in passive mode [to read the colour values from objects that emit light]
        colorSensor3c = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("Color Sensor 3c");
        colorSensor3c.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor3c.enableLed(false);//Put the colour sensor in passive mode [to read the colour values from objects that emit light]
        opticalSensor = hardwareMap.opticalDistanceSensor.get("Optical");

        //controllers for colour sensors to enable or disable them
        color3aController = colorSensor3a.getI2cController();
        color3cController = colorSensor3c.getI2cController();
        color3aCallBack = color3aController.getI2cPortReadyCallback(colorSensor3a.getPort());
        color3cCallBack = color3cController.getI2cPortReadyCallback(colorSensor3c.getPort());

        gyro.resetZAxisIntegrator(); //reset gyro heading to 0

        servoBar = hardwareMap.servo.get("ServoBar");
        // lower servo bar - do it first thing so that servo won't burn trying to get to 0 position
        lowerBar();

        // put the servos in correct position
        servoBeaconPad = hardwareMap.servo.get("BeaconPad");
        setPadPosition(15); // to avoid pad covering camera

        kickServo2 = hardwareMap.servo.get("KickServo2");
        kickServo3 = hardwareMap.servo.get("KickServo3");
        kickServo2.setPosition(10.0 / 255);
        kickServo3.setPosition(215.0 / 255);

        //Starting our log
        out.append("Time,Step,Optical Light,Gyro Reading,Distance,Wheel Encoder Position,Voltage,Color3c - R/G/B,Color3a - R/G/B").append("\n");
        telemetryout("Initial: " + (isBlue ? "blue; " : "red; ") + startTile +
                "; delay: " + delay + "; after shoot: " + afterShootingBehavior);
        telemetry.update();

        // Wait for the game to start (display sensor values)
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
            // reset gyro before we move
            gyro.resetZAxisIntegrator(); //reset gyro heading to 0
            sleep(50);

            // disable color: color sensors are only enabled when detecting color
            // this is done to improve the update rate of all other sensors
            color3aController.deregisterForPortReadyCallback(colorSensor3a.getPort());
            color3cController.deregisterForPortReadyCallback(colorSensor3c.getPort());
            colorSensorsEnabled = false;

            if (TESTING) {
                testSequence();
            } else {
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
            }
        } finally {
            try {
                String name = TESTING ? "testrun" : "lastrun";
                //log file without the time stamp to find it easier
                File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/"+name+".txt");

                //saving the log file into a file
                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write(out.toString());
                outputStreamWriter.close();

                //log file with the time stamp for history
                String timestamp = new SimpleDateFormat("MMMdd_HHmm", Locale.US).format(new Date());
                file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/"+name+"_" + timestamp + ".txt");
                telemetry.addData("File", file.getAbsolutePath());

                //saving the log file into a file
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

        //Variables for the angle change and expected angle
        int angle, fromAngle;

        //start shooting at the position1 (3rd tile from the corner)
        moveOnForShooting(0.5, SHOOT_DISTANCE1, SHOOT_POWER1); //start flywheel and move forward
        motorBelt.setPower(1); //turn on belt and shoot balls
        sleep(2000); //wait 2.5 seconds for balls to shoot
        motorBelt.setPower(0); //stop belt
        motorFlywheel.setPower(0); //turn off flywheels

        telemetryout("After shooting");

        if (stopAfterShooting) { //option
            moveByInches(11); //move 11 inches forward
            powerMotors(0, 0); //stop the robot from moving
            showTelemetry(); //update telemetry
            return;
        }
        if (afterShootingBehavior.equals("rampPark")) {  //option
            parkOnRamp(); //park on the ramp
            return;
        }
        // blue: rotate cc from heading 0
        // red: rotate ccw from heading 0
        angle = isBlue ? 43 : -47;
        rotate(angle, 0);

        telemetryout("After rotated " + angle + " degrees");
        // make sure the robot has settled to get correct heading
        sleep(170); // to make sure gyro has updated

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
        // blue: rotate CW from expected heading
        // red: rotate CCW from expected heading
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
            boolean atWall = driveUntilHit(5, -0.3);
            if (atWall) {
                telemetryout("At the wall");
                // shimmy on the beacon side
                // shimmy(beaconSide);
                moreShimmyIfNeeded();
                telemetryout("After shimmy");
                //moveByInches(8); //move forward eight inches, away from the beacon
                moveByInchesGyro(DRIVING_POWER, isBlue ? 90 : -90, 8, DRIVING_POWER);
                powerMotors(0, 0);
            }
        } else {
            kickServosIn(false); //do not want to wait for the kick servos to return all the way in
            //We move only four since we did not go all the way to the beacon, so we have to back up less.
            //moveByInches(4); //move forward, away from the beacon four inches
            moveByInchesGyro(DRIVING_POWER, isBlue ? 90 : -90, 4, DRIVING_POWER);
            powerMotors(0,0);
        }

        telemetryout("Moved back 8 inches");

        // not using the colour sensors any longer, so we disable them
        color3aController.deregisterForPortReadyCallback(colorSensor3a.getPort());
        color3cController.deregisterForPortReadyCallback(colorSensor3c.getPort());
        colorSensorsEnabled = false;

        // blue: rotate CW from expected heading
        // red: rotate CCW from expected heading
        fromAngle = isBlue ? 90 : -90;
        angle = isBlue ? -88 : 88; // 90 degree turn usually makes 92
        rotate(angle, fromAngle);

        telemetryout("Rotated along the wall");

        liftBar(); //lift bar to repel any potential cap balls

        sleep(170); // to make sure gyro has updated for the next step where we move with gyro

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
        // if we miss white line, we don't want optical sensor to detect red
        // so the distance when we search for white line is limited to 15 inches
        foundWhiteLine = driveUntilWhite(-0.3, 0, 15);
        telemetryout("Found white line 2: " + foundWhiteLine);
        if (!foundWhiteLine) {
            // if line is not detected stop and
            // show telemetry while op mode is active
            powerMotors(0, 0);
            lowerBar();
            showTelemetry();
            return;
        } else {
            // go half robot length past white line
            pastLineInches = 8 + 5; // extra 5 inches to clear space for the turn
            //moveByInches(-pastLineInches);
            moveByInchesGyro(-DRIVING_POWER, 0, pastLineInches, -DRIVING_POWER);
            powerMotors(0,0);
            lowerBar();
            double distanceToBack = 3;
            //moveByInches(distanceToBack);
            moveByInchesGyro(DRIVING_POWER, 0, distanceToBack, DRIVING_POWER);
            powerMotors(0,0);

        }

        telemetryout("Passed " + pastLineInches + " inches after white line 2");

        // blue: rotate CW from expected heading
        // red: rotate CCW from expected heading
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
            boolean atWall = driveUntilHit(5, -0.3);

            if (atWall) {
                telemetryout("At the wall 2");
                // shimmy on the beacon side
                //shimmy(beaconSide);
                moreShimmyIfNeeded();
                telemetryout("After shimmy 2");
            }
        } else {
            kickServosIn(false);
        }

        // disable color
        color3aController.deregisterForPortReadyCallback(colorSensor3a.getPort());
        color3cController.deregisterForPortReadyCallback(colorSensor3c.getPort());
        colorSensorsEnabled = false;

        motorBrush.setPower(1);

        //first travel 1/10th of a circle with the radius of the tile length
        //red - clockwise; blue - counterclockwise
        movePartCircle(TILE_LENGTH, 1 / 10.0, !isBlue, 1.0);

        angle = isBlue ? 45 : -45;
        moveByInchesGyro(0.94, angle, 44, 0.94);

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
        // red: rotate -35 degrees CCW from heading 0 (because the shooter is on the side)
        angle = isBlue ? 45 : -35;
        rotate(angle, 0);

        //sleeping longer to give flywheel time to settle
        sleep(3000);

        // shooting the ball(s)
        motorBelt.setPower(1);
        sleep(3000);
        motorBelt.setPower(0);
        motorFlywheel.setPower(0);

        if (stopAfterShooting) {
            powerMotors(0, 0);
            showTelemetry();
        } else if (afterShootingBehavior.equals("ballPark")) {
            moveBallAndPark();
        } else if (afterShootingBehavior.equals("rampPark")) {
            parkOnRamp();
        } else {
            telemetry.addData(afterShootingBehavior, "not supported");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
        }
    }

    /**
     * Rotate the robot front towards the ramp, start the brush and move there.
     * @throws InterruptedException
     */
    void parkOnRamp() throws InterruptedException {
        int fromAngle, angle, distance;
        if (startTile.startsWith("3rd")) {
            fromAngle = 0;
            angle = isBlue ? -75 : 75;
            distance = isBlue ? 45 : 50;
        } else {
            fromAngle = isBlue ? 45 : -35;
            angle = isBlue ? -110 : 107;
            distance = isBlue ? 70 : 75;
        }
        rotate(angle, fromAngle);
        lowerBar();
        //want to remove the small balls that might be blocking our way in the front
        motorBrush.setPower(1);
        moveByInches(distance, 0.5);
    }

    /**
     * Rotate robot front to the center parking, turn on the brush to move the cap ball, and park there after a delay.
     * @throws InterruptedException
     */
     void moveBallAndPark() throws InterruptedException {
        //delay so that we do not have the cap ball potentially interfere with our alliance partners' robot
        sleep((7 - delay) * 1000);
        int fromAngle = isBlue ? 45 : -45;
        int angle = isBlue ? 170 : -170;
        rotate(angle, fromAngle);
        motorBrush.setPower(1);
        moveByInches(1.25 * TILE_LENGTH);
    }

    /**
     * Lift the bar to repel the cap ball.
     */
     void liftBar() {
        servoBar.setPosition(110.0 / 255);
    }

    /**
     * Lower the bar when we are approaching the beacon area.
     */
     void lowerBar() {
        servoBar.setPosition(225.0 / 255);
    }


    /**
     * Kick Servos : control the handles that move the small balls from in front of the beacon.
     * Brings the kick handles back into the robot.
     * @param withDelays if true, waits for the handles to return into the robot
     */
     void kickServosIn(boolean withDelays) {
        kickServo2.setPosition(10.0 / 255);
        kickServo3.setPosition(215.0 / 255);
        if (withDelays) {
            sleep(300);
        }
    }

    /**
     * Takes 600+ mlliseconds for the kick handles to come all the way out.
     * Kicks the handles halfway out to save time.
     */
     void kickServosHalfway() {
        kickServo2.setPosition(40.0 / 255);
        kickServo3.setPosition(185.0 / 255);
    }

    /**
     * We do not use this method anymore since we stagger the kick handles' push out
     * so a small ball is not caught in between.
     */
     void kickServosOut() {
        kickServo2.setPosition(230.0 / 255);
        kickServo3.setPosition(0);
    }

    /**
     * Tiny rotation clockwise or counter clockwise
     * Used to control small adjustments in the robot heading
     * @param direction "clockwise" for a clockwise direction
     * @throws InterruptedException
     */
     void smallrotate(String direction) throws InterruptedException {
        telemetryout("before small rotate " + direction);
        if (direction.equals("clockwise")) {
            powerMotors(0.3, -0.3);
        } else {
            powerMotors(-0.3, 0.3);
        }
        sleep(150);
        powerMotors(0, 0);
        sleep(100);
    }


    /**
     * Small rotate towards the curved part of the beacon to ensure the button is pressed.
     * @param beaconSide side of the beacon
     * @throws InterruptedException
     */
     void shimmy(BeaconSide beaconSide) throws InterruptedException {
        if (beaconSide == BeaconSide.left) {
            powerMotors(0.3, -0.3);
        } else {
            powerMotors(-0.3, 0.3);
        }
        sleep(150);
        powerMotors(0, 0);
        sleep(100);
    }

    /**
     * If the beacon button is still not pressed, repeat the shimmy towards the curved part of the beacon.
     * @throws InterruptedException
     */
     void moreShimmyIfNeeded() throws InterruptedException {
        BeaconSide beaconSide;
        while (opModeIsActive()) {
            // check color, if detected, keep shimmying
            beaconSide = detectColor(false);
            if (beaconSide == BeaconSide.none) {
                return;
            }
            // shimmy on the beacon side for the second time
            shimmy(beaconSide);
        }
    }


    /**
     * Returns the side of the beacon with our alliance colour.
     * If colour detected successfully, sets the pad position to hit the said colour's button.
     * The robot uses two colour sensors to reliably detect the beacon colour.
     * A colour is detected when one sensor detects more blue than red and the other sensor
     * detects more red than blue. If only one sensor "sees" a colour and the other colour is uncertain,
     * then the robot will make a small rotation towards the uncertain sensor and try to detect
     * colours again. The robot will repeat this process until the colour is detected or the
     * gyro tolerance is exceeded.
     * @param handleUnknown if true, handles situations where one colour sensor falls on the middle,
     * non-specific coloured part of the beacon
     * @return side of the beacon with our alliance colour
     * @throws InterruptedException
     */
     BeaconSide detectColor(boolean handleUnknown) throws InterruptedException {

        int padPosition = 0;
        // detect color (red alliance
        BeaconSide beaconSide = BeaconSide.none;
        boolean colorDetected = false;
        boolean maybe = true; //if one colour sensor detected colour, but the other one did not
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
                    //3a colour is known, and 3c is unknown
                    // rotate right
                    smallrotate("clockwise");
                    maybe = true;
                } else if ((colorSensor3c.red() > colorSensor3c.blue() && colorSensor3a.blue() == colorSensor3a.red()) ||
                        (colorSensor3a.red() == colorSensor3a.blue() && colorSensor3c.blue() > colorSensor3c.red())) {
                    //3c is known, and 3a is unknown
                    //rotate left
                    smallrotate("counterclockwise");
                    maybe = true;
                }
                if (Math.abs(Math.abs(getGyroRawHeading()) - 90) > 10) {
                    telemetryout("gyro tolerance exceeded");
                    maybe = false; //do not want to handle the unknown when the gyro tolerance is exceeded
                }
            }
            telemetry();
        }

        if (colorDetected) {
            //set the pad position
            beaconSide = padPosition == 15 ? BeaconSide.left : BeaconSide.right;
        }
        telemetryout("Color detected: " + beaconSide);

        return beaconSide;
    }

    /**
     * Used to press the beacon button. Robot approaches the wall until a specified distance.
     * @param distancecm how close to the wall the robot should come
     * @param drivingPower the power put into the wheels
     * @return true if the robot could approach the wall, false if not
     * @throws InterruptedException
     */
     boolean driveUntilHit(int distancecm, double drivingPower) throws InterruptedException {

        //makes sure the area before the beacon is clear of small balls
        // we need to delay 450ms for pad to get into position
        // kick servos go in one after the other to shift middle ball from the middle
        kickServo2.setPosition(10.0 / 255);
        sleep(250);
        kickServo3.setPosition(215.0 / 255);
        sleep(200);

        powerMotors(drivingPower, drivingPower);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //time out if robot cannot approach wall
        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > distancecm) {
            if (timer.milliseconds() > 2000) {
                powerMotors(0, 0);
                return false;
            }
        }
        powerMotors(0, 0);
        sleep(600);
        return true;
    }

    /**
     * Set beacon pushing pad position
     * @param pos servo position from zero to 255
     */
     void setPadPosition(int pos) {
        servoBeaconPad.setPosition(pos / 255.0);
    } // from 0 to 255

    /**
     * Shows the sensor readings on the driver station, and continues to update them as we run.
     * @throws InterruptedException
     */
     void showTelemetry() throws InterruptedException {
        while (opModeIsActive()) {
            telemetry();
        }
    }

    /**
     * Shows the sensor readings in the telemetry file.
     */
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
        telemetry.update();
    }

    /**
     * Add the log line of the telemetry [includes all sensor readings]
     * @param step description of the current step.
     */
     void telemetryout(String step) {
        out.append("\n").append(new SimpleDateFormat("MMMdd_HHmm:ss.S", Locale.US).format(new Date())).append(",");
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


    /**
     * Powers the drive motors. Our drive motors are ganged. Whatever goes to the first motor goes
     * to the second motor on the same side.
     * @param leftForward forward power for the left side motors
     * @param rightForward forward power for the right side motors
     * @throws InterruptedException
     */
     void powerMotors(double leftForward, double rightForward) throws InterruptedException {
        // the left side direction is reversed
        motorLeft1.setPower(-leftForward);
        motorRight1.setPower(rightForward);
        idle();
    }

    /**
     * Set run mode (by speed, power, or position) for the drive motors
     * @param runMode DcMotor run mode
     */
     void setRunMode(DcMotor.RunMode runMode) {
        motorLeft1.setMode(runMode);
        motorRight1.setMode(runMode);
    }

    /**
     * When we set zero power to the drive motors, do we want them to break,
     * or keep rotating until friction stops them?
     * @param behavior break or float
     */
     void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        motorLeft1.setZeroPowerBehavior(behavior);
        motorRight1.setZeroPowerBehavior(behavior);
    }

    /**
     * Get autonomous options, which are stored as shared preferences on the phone
     * @param hardwareMap object that stores hardware devices
     * @return preference data
     */
     static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }

    /**
     * Move robot by a given number of inches with the default power
     * @param inches number of inches
     * @throws InterruptedException
     */
     void moveByInches(double inches) throws InterruptedException {
        moveByInches(inches, DRIVING_POWER);
    }

    /**
     * Move robot by a given number of inches with the given power
     * @param inches number of inches
     * @param drivingPower driving power
     * @throws InterruptedException
     */
     void moveByInches(double inches, double drivingPower) throws InterruptedException { // moves 26.5 in one rotation
        telemetry();
        int counts = motorRight1.getCurrentPosition();
        double sign = Math.round(inches / Math.abs(inches));
        powerMotors(sign * drivingPower, sign * drivingPower);
        while (opModeIsActive() && Math.abs(motorRight1.getCurrentPosition() - counts) < inchesToCounts(inches)) {
            telemetry.addData("Raw heading", getGyroRawHeading());
            telemetry.update();
        }
        powerMotors(0, 0);
    }

    /**
     * Rotate robot from current position by an angle with the default driving power.
     * Gyro sensor update rate is too slow, so we use wheel encoder counts to track rotation.
     * In order to avoid accumulated errors, we compare our current robot heading with the expected
     * heading and adjust the rotation angle for the difference.
     * @param angle how many degrees to rotate (difference between expected old and new heading)
     * @param fromRawHeading expected gyro heading,
     * @throws InterruptedException
     */
     void rotate(int angle, int fromRawHeading) throws InterruptedException {
        rotate(angle, fromRawHeading, DRIVING_POWER);
    }

    /**
     * Rotate robot from current position by an angle with the given driving power.
     * Gyro sensor update rate is too slow, so we use wheel encoder counts to track rotation.
     * In order to avoid accumulated errors, we compare our current robot heading with the expected
     * heading and adjust the rotation angle for the difference.
     * @param angle how many degrees to rotate (difference between expected old and new heading)
     * @param fromRawHeading expected gyro heading
     * @param power driving power for rotation
     * @throws InterruptedException
     */
     void rotate(int angle, int fromRawHeading, double power) throws InterruptedException {
        sleep(100); // to make sure robot stopped moving because we are taking gyro reading

        int counts = motorRight1.getCurrentPosition();

        // make an adjustment for the error in current gyro heading
        // pcircle - part of the circle, positive - clockwise
        double pcircleError = getRawHeadingError(fromRawHeading) / 360.0;
        double pcircle = angle / 360.0;
        pcircle = pcircle - pcircleError;
        telemetry.addData("pcircle", pcircle * 360);
        telemetry.update();

        double sign = Math.round(pcircle / Math.abs(pcircle));
        powerMotors(sign * power, -sign * power);
        double circumference = 2 * Math.PI * HALF_WIDTH;
        while (opModeIsActive() && Math.abs(motorRight1.getCurrentPosition() - counts) < Math.abs(pcircle) * inchesToCounts(circumference)) {
            //continue;
        }
        powerMotors(0, 0);
    }

    /**
     * Convert inches into wheel encoder counts.
     * @param inches number of inches
     * @return encoder counts
     */
     double inchesToCounts(double inches) {
        //robot moves 26.5 inches in one motor rotation
        //because of the gear ratio, motor and wheel rotation may be different
        return Math.abs(ENCODER_COUNTS_PER_ROTATION * inches / 26.5);
    }

    /**
     * Drive straight using gyro. This method does a gradient at the end from start to end power.
     *
     * @param startPower          - starting power (positive - forward, negative - backward)
     * @param heading             - raw gyro heading in degrees
     * @param maxInches           - maximum inches to travel
     * @param endPower            - Ending desired power
     * @return true if traveled distance, otherwise false
     * @throws InterruptedException
     */
     boolean moveByInchesGyro(double startPower, int heading, double maxInches, double endPower) throws InterruptedException {

        int initialcount = motorLeft1.getCurrentPosition();
        double error, steerSpeed; // counterclockwise speed
        double inchesForGradient = 5;
        double kp = 0.033; //experimental coefficient for proportional correction of the direction
        double countsSinceStart = Math.abs(motorLeft1.getCurrentPosition() - initialcount);
        double slope = (endPower - startPower) / inchesToCounts(Math.min(inchesForGradient, maxInches)); // this slope is for calculating power
        double countsForGradient = (maxInches < inchesForGradient) ? 0 : Math.abs(inchesToCounts(maxInches - inchesForGradient));
        double motorPower = startPower;
        while (opModeIsActive() && countsSinceStart < inchesToCounts(maxInches)) {
            // error CCW - negative, CW - positive
            error = getRawHeadingError(heading);

            if (Math.abs(error) < 1) {
                steerSpeed = 0;
            } else {
                steerSpeed = kp * error / 4;
            }
            telemetry.addData("Error", error);
            telemetry.update();

            if (countsSinceStart > countsForGradient) {
                motorPower = slope * (countsSinceStart - inchesToCounts(maxInches)) + endPower;
            }
            powerMotors(Range.clip(motorPower - steerSpeed, -1.0, 1.0), Range.clip(motorPower + steerSpeed, -1.0, 1.0));

            countsSinceStart = Math.abs(motorLeft1.getCurrentPosition() - initialcount);
        }
        return opModeIsActive();
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
     boolean driveUntilWhite(double drivingPower, int headingToBeaconZone, double maxInches) throws InterruptedException {

        int initialcount = motorLeft1.getCurrentPosition();
        double error, clockwiseSpeed;
        double kp = 0.033; //experimental coefficient for proportional correction of the direction
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        double light = opticalSensor.getLightDetected();
        while (opModeIsActive() && light < MID_POINT_LIGHT_BACK && Math.abs(motorLeft1.getCurrentPosition() - initialcount) < inchesToCounts(maxInches)) {
            // error CCW - negative, CW - positive
            error = getRawHeadingError(headingToBeaconZone);
            //don't do any correction
            //if heading error < 1 degree
            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else {
                clockwiseSpeed = kp * error / 4;
            }
            telemetry.addData("Error", error);
            telemetry.update();

            powerMotors(drivingPower - clockwiseSpeed, drivingPower + clockwiseSpeed);
            light = opticalSensor.getLightDetected();
        }
        return light >= MID_POINT_LIGHT_BACK;
    }

    /**
     * Rotates robot towards the white line and follows line toward beacon
     * @param drivingPower driving power
     * @param targetWhiteValue value that separates gray from white
     * @return beacon side (currently always none) at which the alliance color was detected (before line follow)
     * @throws InterruptedException
     */
     BeaconSide followLine(double drivingPower, double targetWhiteValue) throws InterruptedException {

        //prepare kick handles to be deployed
        kickServosHalfway();

        double error, clockwiseSpeed;
        double kp = 0.065; //experimental coefficient for proportional correction of the direction
        double light = opticalSensor.getLightDetected();

        // rotate robot toward white line
        if (opModeIsActive() && light < targetWhiteValue) {
            if (isBlue) {
                powerMotors(0.15, -0.15);
            } else {
                powerMotors(-0.15, 0.15);
            }
            light = opticalSensor.getLightDetected();
            while (opModeIsActive() && light < targetWhiteValue) {
                light = opticalSensor.getLightDetected();
            }
            powerMotors(0, 0);
        }

        telemetryout("Stopped at line");

        // move back to the edge of the white line
        //we want the robot to point toward the edge of the white line we're going to follow
        if (opModeIsActive()) {
            if (isBlue) {
                powerMotors(-0.12, 0.12);
            } else {
                powerMotors(0.12, -0.12);
            }
            sleep(25); // let it move a bit back
            light = opticalSensor.getLightDetected();
            while (opModeIsActive() && light < targetWhiteValue) {
                light = opticalSensor.getLightDetected();
            }
            powerMotors(0, 0);
            telemetryout("Moved back a bit");
        }

        //get current distance from wall
        double distance = rangeSensor.getDistance(DistanceUnit.CM);

        //disable gyro sensor update to increase the update rate of the other sensors,
        // including optical distance sensor
        I2cController gyroController = gyro.getI2cController();
        I2cController.I2cPortReadyCallback gyroCallBack = gyroController.getI2cPortReadyCallback(gyro.getPort());
        gyroController.deregisterForPortReadyCallback(gyro.getPort());

        //follow line until robot is 15 cm away from the wall
        while (opModeIsActive() && distance > 15) {

            light = opticalSensor.getLightDetected();

            error = light - targetWhiteValue;
            clockwiseSpeed = kp * error / 0.5;

            if (!isBlue) {
                clockwiseSpeed = -clockwiseSpeed;
            }
            //out.append(light).append(",").append(error).append("\n");

            telemetry.addData("Error", error);
            telemetry.update();

            powerMotors(drivingPower - clockwiseSpeed, drivingPower + clockwiseSpeed);
            distance = rangeSensor.getDistance(DistanceUnit.CM);
        }
        powerMotors(0, 0);

        kickServo2.setPosition(230.0 / 255); // trying to save time

        //enable gyro sensor again
        gyroController.registerForI2cPortReadyCallback(gyroCallBack, gyro.getPort());
        sleep(150); //wait for gyro headings to update to the current value

        //if we are twisted, adjust robot heading
        adjustRobotAngle();

        //enable color sensors
        color3aController.registerForI2cPortReadyCallback(color3aCallBack, colorSensor3a.getPort());
        color3cController.registerForI2cPortReadyCallback(color3cCallBack, colorSensor3c.getPort());
        colorSensorsEnabled = true;
        sleep(150); //wait for color sensors to update to the current values

        kickServo3.setPosition(0); // stagger second kicker

        sleep(500); //TODO: ORIGINAL VALUE ZERO, TESTED TO SEE WHAT WOULD HAPPEN IF SERVOS DEPLOYED ENTIRELY AND NOT JUST HALFWAY

        return BeaconSide.none;
    }

    /**
     * Return current robot heading. This is integrated (accumulated) heading.
     * It doesn't turn to 0 when robot makes a 360 turn.
     * When robot turns clockwise, the angle increases.
     * When the robot turns counterclockwise, the angle decreases.
     * @return  return gyro's integrated heading
     */
     int getGyroRawHeading() {
        //getIntegratedZValue is positive when moving ccw.
        // We want it to behave the same way as getGyroHeading, so we changed the sign.
        return -gyro.getIntegratedZValue();
    }

    /**
     * Get robot heading error, cc error is positive, ccw error is negative
     * @param requiredRawHeading expected robot heading
     * @return error between current and expected robot heading
     */
     double getRawHeadingError(double requiredRawHeading) {
         // difference between current and expected heading
        return getGyroRawHeading() - requiredRawHeading;
    }

    /**
     * Start flywheel and move to shooting position.
     * @param power driving power
     * @param distance distance robot moves to shooting position
     * @param shootPower shooting power, should be negative
     * @throws InterruptedException
     */
     void moveOnForShooting(double power, double distance, double shootPower) throws InterruptedException {

        //modification for additional weight on flywheel
         //Found that instead of -0.1, -0.3 initial speed creates a smaller settling time & lower variation in speed
        double startShootPower = -0.3;
        long rampTime = 2300; // best for linear to avoid the humps and dips in the flywheel speed

        motorFlywheel.setPower(startShootPower); //turn on the flywheel
        telemetryout("Flywheel start");

        int startPosition = motorLeft1.getCurrentPosition(); //calculate start position of robot
        this.powerMotors(-power, -power); //robot starts moving forward
        int currentPosition = startPosition; //current position is the start position

        //use linear ramp to gradually increase flywheel speed to reduce settling time
        long t0 = System.currentTimeMillis(); //calculate start time (milliseconds)
        long currentms = t0; //current time is start time (milliseconds)
        while (opModeIsActive() && ((currentms - t0) < rampTime || Math.abs(currentPosition - startPosition) < inchesToCounts(distance))) {
            if (currentms - t0 < rampTime) {
                // linear
                double flywheelPower = startShootPower + (shootPower - startShootPower) * (currentms - t0) / rampTime;
                motorFlywheel.setPower(flywheelPower);
            } else {
                motorFlywheel.setPower(shootPower);
            }

            //stop robot if it reaches the shooting position
            if (Math.abs(currentPosition - startPosition) >= inchesToCounts(distance)) {
                powerMotors(0, 0);
            }
            currentms = System.currentTimeMillis();
            currentPosition = motorLeft1.getCurrentPosition();
        }
        motorFlywheel.setPower(shootPower); //set power of flywheel to final power
        powerMotors(0, 0); //stop robot if it didn't stop before
        telemetryout("Stopped for shooting");
    }

    /**
     * method for moving robot part of circle
     *
     * @param radius    - radius of circle in inches
     * @param pCircle   - fraction of circle
     * @param clockwise - moving clockwise?
     * @param power     - power to outer wheel
     * @throws InterruptedException
     */
    void movePartCircle(double radius, double pCircle, boolean clockwise, double power) throws InterruptedException {
        double outerWheelDist = 2 * Math.PI * (radius + HALF_WIDTH) * pCircle;
        double powerRatio = (radius - HALF_WIDTH) / (radius + HALF_WIDTH);
        DcMotor outerMotor;
        int counts;
        if ((!clockwise && power > 0) || (clockwise && power < 0)) {
            outerMotor = motorRight1;
            powerMotors(powerRatio * power, power);
        } else {
            outerMotor = motorLeft1;
            powerMotors(power, powerRatio * power);

        }
        counts = outerMotor.getCurrentPosition();

        while (opModeIsActive() && Math.abs(outerMotor.getCurrentPosition() - counts) < inchesToCounts(outerWheelDist)) {
            telemetry.addData("Raw heading", getGyroRawHeading());
            telemetry.update();
        }
    }

    /**
     * If robot heading error is too big to detect white line,
     * adjust robot heading to be within the 10 degrees tolerance.
     * @throws InterruptedException
     */
     void adjustRobotAngle() throws InterruptedException {
        int tolerance = 10;
        int currentHeading = getGyroRawHeading();
        int expectedValue = isBlue ? 90 : -90;
        int currentDiff = currentHeading - expectedValue;
        while (Math.abs(currentDiff) > tolerance) {
            if (currentHeading > expectedValue) {
                smallrotate("counterclockwise");
            } else {
                smallrotate("clockwise");
            }
            currentDiff = getGyroRawHeading() - expectedValue;
        }
    }
}