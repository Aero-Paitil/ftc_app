package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Brandon Lee on 10/30/2016.
 * Updated by Athena 10/30/2016 - rotate to find an image and detect robot location
 * Updated by Athena 11/05/2016 - rotate to an angle
 * Updated by Sangmin and Jasmine 11/05/2016 - drive straight using gyro
 */

@Autonomous(name="Autonomous Mode", group="nb")
public class AutonomousMode extends LinearOpMode {

    private static final float FIELD_WIDTH = 2743.2f; //3580.0f; // millimeter
    private static final float IMAGE_HEIGHT_OVER_FLOOR = 146.05f; // millimeter

    private final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;

    private static final String TAG = "Autonomous Mode";

    private VuforiaTrackables allImages;
    private List<VuforiaTrackable> allTrackables;

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private ColorSensor colorSensorBottom;
    private ColorSensor colorSensor3c;
    private ColorSensor colorSensor3a;

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
    private ModernRoboticsI2cGyro gyro    = null;

    private double [] robotLocation = null;


    @Override
    public void runOpMode() throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("Gyro Sensor ");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
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

        // reset encoders
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // use running by speed mode (to make robot move straight, when equal power is applied left and right)
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range Sensor");
        colorSensorBottom = hardwareMap.colorSensor.get("Color Sensor 3e");
        colorSensorBottom.setI2cAddress(I2cAddr.create8bit(0x3e));
        colorSensorBottom.enableLed(true);
        colorSensor3a = hardwareMap.colorSensor.get("Color Sensor 3a");
        colorSensor3a.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor3a.enableLed(false);
        colorSensor3c = hardwareMap.colorSensor.get("Color Sensor 3c");
        colorSensor3c.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor3c.enableLed(false);

        vuforiaInit();

        //waitForStart();
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", getGyroHeading());
            telemetry.addData(">", "Robot Raw Heading = %d",getGyroRawHeading());
            telemetry.addData("bottom alpha", colorSensorBottom.alpha());
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle();
        }
        //motorBelt.setPower(0.5); // the box is too small at the moment
        servoBeaconPad = hardwareMap.servo.get("BeaconPad");
        setPadPosition((128-15)/3*2+15); // to avoid pad covering camera
        gyro.resetZAxisIntegrator(); //reset gyro heading to 0

        // starting with robot at the wall on the edge of 3rd and 4th tile
        // robot facing backward

        // move forward
        moveByInches(-12);

        // rotate -45 from heading 0
        rotate(-1/8.0, 0);

        // go to white line
        driveUntilWhiteUsingSpeed(-0.5);

        // go 9 inches past white line
        moveByInches(-9);

        // rotate 45 degrees CCW from heading -45
        rotate(-1/8.0, -45);
        telemetry();

        // detect color (red alliance
        boolean colorDetected = false;
        while (!colorDetected && opModeIsActive()) {
            if (colorSensor3a.red() > 0 && colorSensor3c.blue() > 0
                    ) {
                colorDetected = true;
                setPadPosition(15);
            } else if (colorSensor3c.red() > 0 && colorSensor3a.blue() > 0) {
                colorDetected = true;
                setPadPosition(240);
            }
            telemetry();
            idle();
        }

        // if color is detected, move forward to hit the beacon
        if (colorDetected) {
            driveUntilHit(6, -0.3);
        }

        // move 8 inches back
        moveByInches(8);

        // rotate 90 degrees from heading -90
        rotate(1/4.0, -90);

        // drive the distance between two white lines
        moveByInches(-47.5, 0.5);

        // rotate 90 degrees CCW from heading 0
        rotate(-1/4.0, 0);

        // detect color
        colorDetected = false;
        while (!colorDetected && opModeIsActive()) {
            if (colorSensor3a.red() > 0 && colorSensor3c.blue() > 0
                    ) {
                colorDetected = true;
                setPadPosition(15);
            } else if (colorSensor3c.red() > 0 && colorSensor3a.blue() > 0) {
                colorDetected = true;
                setPadPosition(240);
            }
            telemetry();
            idle();
        }

        // if color is detected move to the beacon
        if (colorDetected) {
            driveUntilHit(6, -0.3);
        }

        // move away from beacon
        moveByInches(12);


        //stop tracking images
        allImages.deactivate();

    }

    private void driveUntilHit(int distincm, double drivingPower) throws InterruptedException {
        powerMotors(drivingPower, drivingPower);
        while (rangeSensor.getDistance(DistanceUnit.CM) > distincm){
            idle();
        }
        powerMotors(0,0);
    }

    private void setPadPosition(int pos){
        servoBeaconPad.setPosition(pos/255.0);
    } // from 0 to 255

    public void telemetry() {
        //telemetry.addData("Count: ", motorRight1.getCurrentPosition());
        telemetry.addData("Color 3c - R/G/B: ", "" + colorSensor3c.red() + "/" + colorSensor3c.green() + "/" +
                colorSensor3c.blue() + "     Light reading: " + colorSensor3c.alpha() + "    " +
                " at " + colorSensor3c.getI2cAddress() + " " + colorSensor3c.getConnectionInfo());
        telemetry.addData("Color 3a - R/G/B: ", "" + colorSensor3a.red() + "/" + colorSensor3a.green() + "/" +
                colorSensor3a.blue() + "     Light reading: " + colorSensor3a.alpha() + "     " +
                " at " + colorSensor3a.getI2cAddress() + " " + colorSensor3a.getConnectionInfo());
        telemetry.addData("Color Bottom = R/G/B", colorSensorBottom.alpha() +
                " at " + colorSensorBottom.getI2cAddress() + " " + colorSensorBottom.getConnectionInfo());
        telemetry.addData(">", "Heading = %d", getGyroHeading());
        telemetry.addData(">", "Gyro Reading = %d", getGyroRawHeading());
        if (robotLocation != null && robotLocation.length == 3) {
            telemetry.addData("Location", "x = %.0f, y = %.0f, z = %.0f", robotLocation[0], robotLocation[1], robotLocation[2]);
        }
        telemetry.update();
    }

    private void powerMotors(double leftForward, double rightForward) throws InterruptedException {
        // the left side direction is reversed
        motorLeft1.setPower(-leftForward);
        motorRight1.setPower(rightForward);
        idle();
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        motorLeft1.setMode(runMode);
        motorRight1.setMode(runMode);
    }

    private void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        motorLeft1.setZeroPowerBehavior(behavior);
        motorRight1.setZeroPowerBehavior(behavior);
    }

    private void vuforiaInit(){
        try{
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            SharedPreferences prefs = getSharedPrefs(hardwareMap);

            parameters.vuforiaLicenseKey = prefs.getString("vuforiaLicenseKey", null);
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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
                    .translation(FIELD_WIDTH /12, FIELD_WIDTH /2, IMAGE_HEIGHT_OVER_FLOOR)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            wheelTarget.setLocation(wheelTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(wheelTargetLocationOnField), wheelTarget.getName());

            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELD_WIDTH /4, FIELD_WIDTH /2, IMAGE_HEIGHT_OVER_FLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            legosTarget.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(legosTargetLocationOnField), legosTarget.getName());

            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELD_WIDTH /2, FIELD_WIDTH /4, IMAGE_HEIGHT_OVER_FLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            toolsTarget.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(toolsTargetLocationOnField), toolsTarget.getName());

            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELD_WIDTH /2, -FIELD_WIDTH /12, IMAGE_HEIGHT_OVER_FLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            gearsTarget.setLocation(gearsTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(gearsTargetLocationOnField), gearsTarget.getName());

            OpenGLMatrix phoneLocationOnRobot =  OpenGLMatrix.translation(0,0,0);
//                        .translation(209.55f, 177.8f, 266.7f) // these are all in mm
//                        .multiplied(Orientation.getRotationMatrix(
//                                AxesReference.EXTRINSIC, AxesOrder.XZX,
//                                AngleUnit.DEGREES, 90, 0, 0));
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
    private void updateRobotLocation(){
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

                    robotLocation = new double[] {x, y, zAngle};
                }
            }
            //telemetry.update();
        } catch (Exception e){
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

    private void moveByInches(double inches, double drivingPower) throws InterruptedException{ // moves 26.5 in one rotation
        telemetry();
        int counts = motorRight1.getCurrentPosition();
        double sign = Math.round(inches/Math.abs(inches));
        powerMotors(sign*drivingPower, sign*drivingPower);
        while (opModeIsActive() && Math.abs(motorRight1.getCurrentPosition() - counts) < Math.abs(ENCODER_COUNTS_PER_ROTATION*inches/26.5)){
            idle();
            //telemetry();
        }
        powerMotors(0,0);
    }

    private void driveUntilWhiteUsingSpeed(double drivingPower) throws InterruptedException{
        powerMotors(drivingPower, drivingPower);
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        double alpha = colorSensorBottom.alpha();
        double distance = rangeSensor.getDistance(DistanceUnit.CM);
        while (opModeIsActive() && alpha < MID_POINT_ALPHA_FRONT && distance > 6){
            // keep going
            idle();
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            alpha = colorSensorBottom.alpha();
        }
        powerMotors(0,0);
    }


    // pcircle - part of the circle, positive - clockwise
    private void rotate(double pcircle, int fromRawHeading) throws InterruptedException {
        int counts = motorRight1.getCurrentPosition();

        // make an adjustment for the error in current gyro heading
        double pcircleError = getRawHeadingError(fromRawHeading)/360.0;
        pcircle -= pcircleError;

        double sign = Math.round(pcircle/Math.abs(pcircle));
        // assuming 16 inches between wheels, 8 inches radius - 50.24 in
        // assuming 15 inches between wheels, 7.5 inches radius - 46.24 in
        // assuming 15.5 inches between wheels, 7.75 inches radius - 48.67 in
        while (Math.abs(motorRight1.getCurrentPosition() - counts) < Math.abs(pcircle*ENCODER_COUNTS_PER_ROTATION*48.67/26.5)){
            powerMotors(sign*DRIVING_POWER, -sign*DRIVING_POWER);
        }
        powerMotors(0,0);
    }

    private double DRIVING_POWER = 0.3;
    private int MID_POINT_ALPHA_FRONT = 5;
    //private double MAX_COUNTS_TO_WHITE = 2.26 * ENCODER_COUNTS_PER_ROTATION; // 4.5 to reach white line
    private void driveUntilWhite(double drivingPower) throws InterruptedException{
        //gyro.resetZAxisIntegrator();
        double headingToBeaconZone = getGyroHeading();
        powerMotors(drivingPower, drivingPower);
        idle();
        double currentHeading, error, clockwiseSpeed;
        double kp = 0.03 ; //experimental coefficient for proportional correction of the direction
        //alpha() is to measure the brightness.
        //maintain the direction until robot "sees" the edge of white line/touches/close to some other object
        double alpha = colorSensorBottom.alpha();
        double distance = rangeSensor.getDistance(DistanceUnit.CM);
        //double leftcounts = motorLeft1.getCurrentPosition();
        while (opModeIsActive() && alpha < MID_POINT_ALPHA_FRONT){
            // keep going
            currentHeading = getGyroHeading();
            error = getHeadingDelta(headingToBeaconZone);
            if  (headingToBeaconZone>currentHeading || headingToBeaconZone-currentHeading<0) {
                error = -error;
            }
            //don't do any correction
            //if heading error < 1 degree
            if (Math.abs(error) < 1) {
                clockwiseSpeed = 0;
            } else if (Math.abs(error) >= 1 && Math.abs(error) <= 4) {
                clockwiseSpeed = kp * error / 4;
            } else {
                clockwiseSpeed = kp * Math.abs(error) / error;
            }
            //clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            distance = rangeSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Distance", distance);

            telemetry.addData("LeftCounts", motorLeft1.getCurrentPosition());
            telemetry.addData("RightCounts", motorRight1.getCurrentPosition());

            telemetry.update();
            //DbgLog.msg(i + " clockwise speed "+clockwiseSpeed);
            double sign = Math.round(drivingPower/Math.abs(drivingPower));
            powerMotors(drivingPower - clockwiseSpeed * sign, drivingPower + clockwiseSpeed * sign);

            idle();
            //sleep(25);
            alpha = colorSensorBottom.alpha();
        }

        motorBrush.setPower(0);
        powerMotors(0,0);
    }

    private int getGyroHeading() {
        return gyro.getHeading();
    }

    //getIntegratedZValue is positive when moving ccw. We want it to behave the same way as getGyroHeading, so we changed the sign.
    private int getGyroRawHeading() {return -gyro.getIntegratedZValue();}

    // gyro heading is from 0 to 359
    private void rotateToHeading(double requiredHeading) throws InterruptedException {

        double MIN_ROTATE_POWER = 0.15;

        // make sure requiredHeading is positive and less than 360
        while (requiredHeading >= 360) {
            requiredHeading -= 360;
        }
        while (requiredHeading < 0) {
            requiredHeading += 360;
        }

        double power;
        // choose rotation direction
        double currentHeading = getGyroHeading();
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
        //rotateToTolerance(60, requiredHeading, power * 1);
        //rotateToTolerance(40, requiredHeading, power * 0.7);
        rotateToTolerance(20, requiredHeading, power);
        powerMotors(0, 0);
        idle();
    }

    // we want the delta to be less than 180 degrees
    // for example, the delta between 350 and 10 should be 20
    private double getHeadingDelta(double requiredHeading) {
        double headingDelta = Math.abs(requiredHeading - getGyroHeading());
        if (headingDelta > 180) {
            headingDelta = 360 - headingDelta;
        }
        return headingDelta;
    }

    //cc error is positive, ccw error is negative
    private double getRawHeadingError(double requiredRawHeading) {
        return getGyroRawHeading() - requiredRawHeading;
    }

    // remember to zero the power after this method call if you want to stop rotation
    private void rotateToTolerance(double tolerance, double requiredHeading, double power) throws InterruptedException {
        double headingDelta = getHeadingDelta(requiredHeading);
        double lastHeadingDelta = headingDelta;

        if (headingDelta > tolerance) {
            // power motors
            powerMotors(power, -power);
            idle();
            // if current gyro heading is not close enough to the required heading
            // wait and check again
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (headingDelta > tolerance && headingDelta <= lastHeadingDelta + 1 && opModeIsActive()) {
                //checkTimeout(timer, 10);
                idle();
                lastHeadingDelta = headingDelta;
                headingDelta = getHeadingDelta(requiredHeading);
                telemetry.addData("delta", headingDelta);
                telemetry.addData(">", "Rotating = %d", getGyroHeading());
                telemetry.update();
            }
        }
    }


// constants from sample code for using gyro turn/drive
//    // These constants define the desired driving/control characteristics
//    // The can/should be tweaked to suite the specific robot drive train.
//    static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
//    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.
//
//    static final double     HEADING_THRESHOLD       = 3 ;      // As tight as we can make it with an integer gyro
//    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
//
// sample code - the error is too big - will try our own routines
//    /**
//     * getError determines the error between the target angle and the robot's current heading
//     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
//     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
//     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
//     */
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - gyro.getIntegratedZValue();
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
//
//    /**
//     * returns desired steering force.  +/- 1 range.  +ve = steer left
//     * @param error   Error angle in robot relative degrees
//     * @param PCoeff  Proportional Gain Coefficient
//     * @return
//     */
//    public double getSteer(double error, double PCoeff) {
//        return Range.clip(error * PCoeff, -1, 1);
//    }
//
//    /**
//     * Perform one cycle of closed loop heading control.
//     *
//     * @param speed     Desired speed of turn.
//     * @param angle
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//        powerMotors(leftSpeed, rightSpeed);
//
//        // Display it for the driver.
//        telemetry.addData("Target", "%5.0f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//
//    /**
//     *  Method to spin on central axis to point in a new direction.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the heading (angle)
//     *  2) Driver stops the opmode running.
//     *
//     * @param speed Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     * @throws InterruptedException
//     */
//    public void gyroTurn (  double speed, double angle)
//            throws InterruptedException {
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//            idle();
//        }
//    }


}
