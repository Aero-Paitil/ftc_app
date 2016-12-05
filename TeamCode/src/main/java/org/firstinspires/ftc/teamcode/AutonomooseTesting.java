package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
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

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by Brandon Lee on 10/30/2016.
 * Updated by Athena 10/30/2016 - rotate to find an image and detect robot location
 * Updated by Athena 11/05/2016 - rotate to an angle
 * Updated by Sangmin and Jasmine 11/05/2016 - drive straight using gyro
 */

@Autonomous(name="TheMooseTest", group="nb")
public class AutonomooseTesting extends LinearOpMode {

    private double DRIVING_POWER = 0.3;
    private double MID_POINT_LIGHT_BACK = 0.3;

    private static final float FIELD_WIDTH = 2743.2f; //3580.0f; // millimeter
    private static final float IMAGE_HEIGHT_OVER_FLOOR = 146.05f; // millimeter

    private final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;

    private static final int RED_SHOOT_DISTANCE1 = 21;
    private static final int RED_SHOOT_DISTANCE2 = 21;

    private static final String TAG = "Autonomous Mode";

    private VuforiaTrackables allImages;
    private List<VuforiaTrackable> allTrackables;

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private OpticalDistanceSensor opticalSensor;
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
    //private DcMotor motorFlywheelLeft;
    private DcMotor motorFlywheelRight;
    private DcMotor motorFlywheelLeft;
    private Servo servoBeaconPad;
    private ModernRoboticsI2cGyro gyro    = null;

    StringBuffer out = new StringBuffer();

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
        motorFlywheelRight = hardwareMap.dcMotor.get("GunRight");
        motorFlywheelLeft = hardwareMap.dcMotor.get("GunLeft");
        // run flywheels by speed
        motorFlywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFlywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // reset encoders
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // use running by speed mode (to make robot move straight, when equal power is applied left and right)
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range Sensor");
        opticalSensor = hardwareMap.opticalDistanceSensor.get("Optical");
        colorSensor3a = hardwareMap.colorSensor.get("Color Sensor 3a");
        colorSensor3a.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor3a.enableLed(false);
        colorSensor3c = hardwareMap.colorSensor.get("Color Sensor 3c");
        colorSensor3c.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor3c.enableLed(false);
        gyro.resetZAxisIntegrator(); //reset gyro heading to 0

        //waitForStart();
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry();
            idle();
        }

        // if we hit stop after init, nothing else should happen
        if (!opModeIsActive() ){
            return;
        }

        gyro.resetZAxisIntegrator(); //reset gyro heading to 0
        sleep(50);

        //motorBelt.setPower(0.5); // the box is too small at the moment
        servoBeaconPad = hardwareMap.servo.get("BeaconPad");
        setPadPosition((128-15)/3*2+15); // to avoid pad covering camera

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);


        //testOpticalDistanceSensor();

        followLine(-0.3, 0.35 );

    }

    private void testFlywheels() throws InterruptedException {
        this.powerMotors(-DRIVING_POWER,-DRIVING_POWER);
        int currentPosition =0;
        int startPosition = motorLeft1.getCurrentPosition();

        while((currentPosition - startPosition)< ENCODER_COUNTS_PER_ROTATION*RED_SHOOT_DISTANCE1/26.5){
            currentPosition = motorLeft1.getCurrentPosition();
            idle();
        }
        powerMotors(0,0);

        motorBelt.setPower(0.8);
        powerFlywheels(true);
        sleep(5000);
        motorBelt.setPower(0);
        powerFlywheels(false);
    }

    private void powerFlywheels(boolean doPower) throws InterruptedException {
        // theflywheels should move out in the opposite direction
        if (doPower) {
            motorFlywheelLeft.setPower(1);
            motorFlywheelRight.setPower(-1);
        } else {
            motorFlywheelLeft.setPower(0);
            motorFlywheelRight.setPower(0);
        }
        idle();
    }

    private void testOpticalDistanceSensor() throws InterruptedException {
        int delayMs = 500;

        for (int i = 0; i < 3; i++) {
            gyro.resetZAxisIntegrator();
            rotateOverLine(10, 0.2);
            sleep(delayMs);
            gyro.resetZAxisIntegrator(); //reset gyro heading to 0
            sleep(2000);
            rotateOverLine(-10, 0.2);
            sleep(delayMs);
        }
    }


    /*
     * Returns beacon side at which the alliance color was detected (before line follow)
     */
    private AutonomousMode.BeaconSide followLine(double drivingPower, double targetWhiteValue) throws InterruptedException {
        boolean isBlue = false;
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
            out.append("Moved back a bit");
        }

        double distance = rangeSensor.getDistance(DistanceUnit.CM);

        while (opModeIsActive() && distance > 11) {

            light = opticalSensor.getLightDetected();

            error = light - targetWhiteValue;
            //don't do any correction
            //if  error < 0.1
            if (Math.abs(error) < 0.1) {
                clockwiseSpeed = 0;
            } else {
                clockwiseSpeed = kp * error / 0.5;
            }

            clockwiseSpeed = -clockwiseSpeed;

            out.append(light+","+error+"\n");

            //clockwiseSpeed = Range.clip(clockwiseSpeed, -1.0, 1.0);
            telemetry.addData("Error", error);
            telemetry.update();

            powerMotors(drivingPower - clockwiseSpeed, drivingPower + clockwiseSpeed);
            sleep(5);
            distance = rangeSensor.getDistance(DistanceUnit.CM);
        }
        powerMotors(0, 0);
        return AutonomousMode.BeaconSide.none;
    }


    private int idx =0;

    private void rotateOverLine(double inches, double drivingPower) throws InterruptedException{ // moves 26.5 in one rotation

        int counts = motorRight1.getCurrentPosition();
        double sign = Math.round(inches/Math.abs(inches));
        powerMotors(sign*drivingPower, -sign*drivingPower);
        int counter = 0;
        int iters = 0;
        long cms;
        long maxdiff = 0;
        long sumdiff = 0;
        long firsttime = -1;
        long endtime = -1;
        ArrayList<Integer> position = new ArrayList<>();
        ArrayList<Integer> heading = new ArrayList<>();
        ArrayList<Double> detected = new ArrayList<>();
        ArrayList<Long> currentTimeMs = new ArrayList<>();
        double lightDetected;
        int currPos = Math.abs(motorRight1.getCurrentPosition() - counts);
        long initialMs = System.currentTimeMillis();
        long ms = initialMs;
        long whitetime=0;
        while (opModeIsActive() && currPos < Math.abs(ENCODER_COUNTS_PER_ROTATION*inches/26.5)){
            position.add(currPos);
            cms = System.currentTimeMillis();
            currentTimeMs.add(cms-initialMs);
            lightDetected = opticalSensor.getLightDetected();
            detected.add(lightDetected);
            heading.add(0);
            if ( lightDetected > MID_POINT_LIGHT_BACK){
                counter++;
                if (firsttime == -1){
                    firsttime = System.currentTimeMillis();
                }
            } else {
                if (whitetime == 0 && firsttime > 0){
                    endtime = System.currentTimeMillis();
                    whitetime = endtime-firsttime;
                }
            }
            sumdiff += cms - ms;
            if (cms - ms > maxdiff) maxdiff = cms - ms;
            ms = cms;
            iters++;
            //waitOneFullHardwareCycle();
            idle();
            sleep(5);
            currPos = Math.abs(motorRight1.getCurrentPosition() - counts);
        }
        telemetry.addData("WhiteTime: ", whitetime);
        telemetry.addData("Maxdiff: ", maxdiff);
        telemetry.addData("Avgdiff: ", (double)sumdiff / (iters));
        telemetry.addData("Counter: ", counter);
        telemetry.addData("Iterations: ", iters);

        powerMotors(0,0);

        // write to file when missed line or first time moving in the same direction after missed line

        try {
            File file = new File(Environment.getExternalStorageDirectory().getPath() + (counter == 0 ? "/FIRST/missed" : "/FIRST/detected") + idx + ".txt");
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            // outputStreamWriter.write("cycleMs,posDiff,gyro,detectedLight\n");
            for (int i = 0; i < position.size(); i++) {
                outputStreamWriter.write(currentTimeMs.get(i) + "," + position.get(i) + "," + heading.get(i) +","+ detected.get(i) + "\n");
            }
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }

        telemetry.update();
        idx++;
    }


    private void setPadPosition(int pos){
        servoBeaconPad.setPosition(pos/255.0);
    } // from 0 to 255

    private void showTelemetry() throws InterruptedException {
        while (opModeIsActive()) {
            telemetry();
            idle();
        }
    }

    public void telemetry() {
        telemetry.addData("Color 3c - R/G/B: ", "" + colorSensor3c.red() + "/" + colorSensor3c.green() + "/" +
                colorSensor3c.blue() + "     Light reading: " + colorSensor3c.alpha() + "    " +
                " at " + colorSensor3c.getI2cAddress() + " " + colorSensor3c.getConnectionInfo());
        telemetry.addData("Color 3a - R/G/B: ", "" + colorSensor3a.red() + "/" + colorSensor3a.green() + "/" +
                colorSensor3a.blue() + "     Light reading: " + colorSensor3a.alpha() + "     " +
                " at " + colorSensor3a.getI2cAddress() + " " + colorSensor3a.getConnectionInfo());
        telemetry.addData("Optical Light: ", opticalSensor.getLightDetected());
        telemetry.addData("Gyro", getGyroRawHeading());
//        if (robotLocation != null && robotLocation.length == 3) {
//            telemetry.addData("Location", "x = %.0f, y = %.0f, z = %.0f", robotLocation[0], robotLocation[1], robotLocation[2]);
//        }
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


    // pcircle - part of the circle, positive - clockwise
    private void rotate(int angle, int fromRawHeading) throws InterruptedException {
        sleep(100); // to make sure robot stopped moving

        ArrayList<Long> ms = new ArrayList<Long>();
        ArrayList<Integer> gyroDiffs= new ArrayList<Integer>();
        ArrayList<Double> pcicles = new ArrayList<Double>();
        ArrayList<Integer> gyroReadings = new ArrayList<Integer>();
        ArrayList<Integer> encoderCounts = new ArrayList<Integer>();

        gyro.resetZAxisIntegrator();
        int counts = motorRight1.getCurrentPosition();

        // make an adjustment for the error in current gyro heading
        double pcircleError = getRawHeadingError(fromRawHeading)/360.0;
        double pcircle = angle / 360.0;
        pcircle = pcircle-pcircleError;

        Long startTime = System.currentTimeMillis();
        int startGyro = getGyroRawHeading();
        int curPos = 0;


        double sign = Math.round(pcircle/Math.abs(pcircle));
        powerMotors(sign*DRIVING_POWER, -sign*DRIVING_POWER);

        // assuming 16 inches between wheels, 8 inches radius - 50.24 in
        // assuming 15 inches between wheels, 7.5 inches radius - 46.24 in
        // assuming 15.5 inches between wheels, 7.75 inches radius - 48.67 in
        while (Math.abs(curPos - counts) < Math.abs(pcircle*ENCODER_COUNTS_PER_ROTATION*48.67/26.5)){
            ms.add(System.currentTimeMillis() - startTime);
            gyroReadings.add(new Integer(getGyroRawHeading()));
            int gyroDiff = getGyroRawHeading() - startGyro;
            gyroDiffs.add(new Integer(gyroDiff));
            double curPcircle = angle/360- gyroDiff/360;
            pcicles.add(new Double(curPcircle));
            encoderCounts.add(curPos - counts );
            curPos = motorRight1.getCurrentPosition();

            idle();
        }
        powerMotors(0,0);

        //test, get overshoot final degree, compare with the original target degree and find the adjustment for next movement
        int finalGyroRawHeading = getGyroRawHeading();
        int targetHeadingError = finalGyroRawHeading - (fromRawHeading + angle);


        // write to file when missed line or first time moving in the same direction after missed line
            try {
                File file = new File(Environment.getExternalStorageDirectory().getPath() +  "/FIRST/detected.txt");
                telemetry.addData("File", file.getAbsolutePath());
                telemetry.update();

                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
                outputStreamWriter.write("TimeMs\t gyroReading\t gyroDiffs\t pCicle\t, encoderCounts\n");
                for (int i = 0; i < gyroReadings.size(); i++) {
                    outputStreamWriter.write(ms.get(i) + "\t" + gyroReadings.get(i) + "\t" + gyroDiffs.get(i) +"\t"+ pcicles.get(i) +"\t"+encoderCounts.get(i) + "\n");
                }
                outputStreamWriter.write( "Final targetHeadingError = finalGyroRawHeading - (fromRawHeading + angle )" + targetHeadingError);
                outputStreamWriter.close();

            } catch (Exception e) {
                telemetry.addData("Exception", "File write failed: " + e.toString());
            }
    }


    private int getGyroHeading() {
        return gyro.getHeading();
    }

    //getIntegratedZValue is positive when moving ccw. We want it to behave the same way as getGyroHeading, so we changed the sign.
    private int getGyroRawHeading() {
        return -gyro.getIntegratedZValue();
    }

    //cc error is positive, ccw error is negative
    private double getRawHeadingError(double requiredRawHeading) {
        return getGyroRawHeading() - requiredRawHeading;
    }

}
