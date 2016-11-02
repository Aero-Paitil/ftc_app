package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="DriverMode", group="nb")
//@Disabled

public class DriverMode extends OpMode {

    //public static final String ALLIANCE = "RED";
    private static final float FIELDWIDTH = 2743.2f; //3580.0f; // millimeter
    private static final float IMAGEHEIGHTOVERFLOOR = 146.05f; // millimeter
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime motorleftTimer = new ElapsedTime();
    private ElapsedTime motorrightTimer = new ElapsedTime();
    private double lastLeftForward = 0;
    private double lastRightForward = 0;

    private static final String TAG = "Driver Mode";
    private OpenGLMatrix lastLocation = null;

    //defining the 4 motors
    private DcMotor motorLeft1;
    private DcMotor motorRight1;
    private DcMotor motorLeft2;
    private DcMotor motorRight2;

    @Override
    public void init() {
        //"initializing" the motors
        motorLeft1 = hardwareMap.dcMotor.get("D1left");
        motorRight1 = hardwareMap.dcMotor.get("D1right");
        motorLeft2 = hardwareMap.dcMotor.get("D2left");
        motorRight2 = hardwareMap.dcMotor.get("D2right");

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        vuforiaInit();
    }

    @Override
    public void start() {
        allImages.activate();
    }

    @Override
    public void loop() {

        double leftForward = -gamepad1.left_stick_y;
        double rightForward = -gamepad1.right_stick_y;

        //todo adjust the deadband
        if ((rightForward > -0.1) && (rightForward < 0.1)) rightForward = 0;
        if ((leftForward > -0.1) && (leftForward < 0.1)) leftForward = 0;

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            //We are using robot coordinates

            double dpadSpeed = 0.5;

            if (gamepad1.dpad_up) {
                rightForward = dpadSpeed;
                leftForward = dpadSpeed;
            } else if (gamepad1.dpad_down) {
                rightForward = -dpadSpeed;
                leftForward = -dpadSpeed;
            } else if (gamepad1.dpad_left) {
                rightForward = dpadSpeed;
                leftForward = -dpadSpeed;
            } else {
                leftForward = dpadSpeed;
                rightForward = -dpadSpeed;
            }
        }

        /* assigning the motors the scaled powers that we just calculated in the step above. */
        powerMotors(rightForward, leftForward);

        if (timer.time() > 0.5){
            vuforiaLoop();
        }

        if (gamepad1.x) {
            goToPoint(-FIELDWIDTH / 2, -FIELDWIDTH / 12);
        }
    }

    private void powerMotors(double rightForward, double leftForward) {
        double diffLeft = leftForward - lastLeftForward;
        if (Math.abs(diffLeft) <= 0.2){
            motorLeft1.setPower(leftForward);
            motorLeft2.setPower(leftForward);
            lastLeftForward = leftForward;
            motorleftTimer.reset();
        }
        else if (motorleftTimer.time() >= 0.1) {
            lastLeftForward = lastLeftForward + (0.2*diffLeft)/(Math.abs(diffLeft));
            motorLeft1.setPower(lastLeftForward);
            motorLeft2.setPower(lastLeftForward);
            motorleftTimer.reset();
        }
        double diffRight = rightForward - lastRightForward;
        if (Math.abs(diffRight) <= 0.2){
            motorRight1.setPower(rightForward);
            motorRight2.setPower(rightForward);
            lastRightForward = rightForward;
            motorrightTimer.reset();
        }
        else if (motorrightTimer.time() >= 0.1) {
            lastRightForward = lastRightForward +(0.2*diffRight)/(Math.abs(diffRight));
            motorRight1.setPower(lastRightForward);
            motorRight2.setPower(lastRightForward);
            motorrightTimer.reset();
        }
    }
    private VuforiaTrackables allImages;
    private List<VuforiaTrackable> allTrackables;

    private void goToPoint(double golx, double goly) { // will be interrupted by using D-Pad
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) return;
        double basePower = getBasePower(golx,goly);
        double headingDelta = getHeadingDelta(golx, goly);
        if (headingDelta > 20) {
            double powerAdjust = headingDelta/360;
            motorRight1.setPower(basePower+powerAdjust);
            motorRight2.setPower(basePower+powerAdjust);
            motorLeft1.setPower(basePower-powerAdjust);
            motorLeft2.setPower(basePower-powerAdjust);
        }else {
            motorRight1.setPower(basePower);
            motorRight2.setPower(basePower);
            motorLeft1.setPower(basePower);
            motorRight2.setPower(basePower);
        }
    }
    private double getBasePower(double golx, double goly) {
        VectorF v = lastLocation.getTranslation();
        double currentx = v.get(0);
        double currenty = v.get(1);
        double distance = dist(currentx, currenty, golx, goly);
        if (distance > 640) {
            return 0.5;
        } else if (distance > 30){
            return distance * 0.5/640f;
        }else{
            return 0;
        }
    }
    private double dist(double x, double y, double gx, double gy){
        return (Math.sqrt(
                Math.pow(x-gx, 2) + Math.pow(y-gy, 2)
        ));
    }
    private double getHeadingDelta(double theta){ // this is in degrees
        Orientation ori = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double currenttheta = ori.thirdAngle; // TODO: This code is assuming that this is also on a -180 to 180 scale
        return Math.abs(currenttheta - theta);
    }
    private double getHeadingDelta(double golx, double goly){ // this will do getHeadingDelta(theta) where -180 < theta < 180
        VectorF v = lastLocation.getTranslation();
        double currentx = v.get(0);
        double currenty = v.get(1);
        double theta = Math.sin(currenty/currentx)*180/Math.PI;
        if (golx >= currentx){ // 1st and 4th quadrants
            return getHeadingDelta(theta);
        }else{
            if (goly >= currenty){ // 2nd quadrant
                return getHeadingDelta(theta + 180);
            }else{ // 3rd quadrant
                return getHeadingDelta(theta - 180);
            }
        }
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
                    .translation(FIELDWIDTH/12, FIELDWIDTH/2, IMAGEHEIGHTOVERFLOOR)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            wheelTarget.setLocation(wheelTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(wheelTargetLocationOnField), wheelTarget.getName());

            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELDWIDTH/4, FIELDWIDTH/2, IMAGEHEIGHTOVERFLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            legosTarget.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(legosTargetLocationOnField), legosTarget.getName());

            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELDWIDTH/2, FIELDWIDTH/4, IMAGEHEIGHTOVERFLOOR) // 146.05 is the height of the center of the image over the tile surface.
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 90, 0));
            toolsTarget.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Target=%s %s", format(toolsTargetLocationOnField), toolsTarget.getName());

            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                    .translation(-FIELDWIDTH/2, -FIELDWIDTH/12, IMAGEHEIGHTOVERFLOOR) // 146.05 is the height of the center of the image over the tile surface.
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

    private void vuforiaLoop(){
        timer.reset();
        boolean isVisible;
        try {
            for (VuforiaTrackable trackable : allTrackables) {
                isVisible = ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();
                telemetry.addData(trackable.getName(), isVisible ? "Visible" : "Not Visible");
                if (!isVisible) {
                    continue;
                }

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null && ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    lastLocation = robotLocationTransform;
                }
                if (lastLocation != null) {
                    RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                    telemetry.addData("Pos", format(lastLocation));
                }
            }
            telemetry.update();
        } catch (Exception e){
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

}
