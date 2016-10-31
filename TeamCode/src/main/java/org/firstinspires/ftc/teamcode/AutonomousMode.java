package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

/**
 * Created by Brandon Lee on 10/30/2016.
 * Updated by Athena 10/30/2016 - rotate to find an image and detect robot location
 */

@Autonomous(name="Autonomous Mode", group="nb")
public class AutonomousMode extends LinearOpMode {

    private static final float FIELD_WIDTH = 2743.2f; //3580.0f; // millimeter
    private static final float IMAGE_HEIGHT_OVER_FLOOR = 146.05f; // millimeter
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private static final String TAG = "Autonomous Mode";

    private VuforiaTrackables allImages;
    private List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode() throws InterruptedException {

        //defining the 4 motors
        DcMotor motorLeft1;
        DcMotor motorRight1;
        DcMotor motorLeft2;
        DcMotor motorRight2;

        //"initializing" the motors
        motorLeft1 = hardwareMap.dcMotor.get("D1left");
        motorRight1 = hardwareMap.dcMotor.get("D1right");
        motorLeft2 = hardwareMap.dcMotor.get("D2left");
        motorRight2 = hardwareMap.dcMotor.get("D2right");

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        vuforiaInit();

        waitForStart();

        // start tracking images
        allImages.activate();
        sleep(50);
        idle();

        // first position detection seems to be always null
        double[] robotLocation = getRobotLocation();
        sleep(50);
        idle();

        double rotationPower = 0.15;
        timer.reset();
        timer2.reset();

        while (robotLocation == null && opModeIsActive()) {

            // looks like we never get the position if the phone is constantly moving
            // but we can get position when it just starting to move
            // with 1.5 power, 0.2 sec rest, 0.6 sec move, we can always get the position
            // from the third tile from an image. One iteration with this settings is about 15 degrees.
            if (timer.time()<0.2) {
                motorLeft1.setPower(0);
                motorLeft2.setPower(0);
                motorRight1.setPower(0);
                motorRight2.setPower(0);
            } else if (timer.time()<0.8) {
                //robot rotates until it knows its location
                motorLeft1.setPower(rotationPower);
                motorLeft2.setPower(rotationPower);
                motorRight1.setPower(-rotationPower);
                motorRight2.setPower(-rotationPower);
            } else if (timer.time() >= 0.8) {
                timer.reset();
            }
            sleep(50);
            idle();

            robotLocation = getRobotLocation();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);

        sleep(50);
        idle();

        printRobotLocation("Robot location", robotLocation);

        robotLocation = getRobotLocation();

        printRobotLocation("Final location", robotLocation);
        telemetry.update();

        //stop tracking images
        allImages.deactivate();

        while (opModeIsActive()) {
            idle();
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
    private double[] getRobotLocation(){
        boolean isVisible;
        VuforiaTrackableDefaultListener listener;
        try {
            for (VuforiaTrackable trackable : allTrackables) {
                listener = ((VuforiaTrackableDefaultListener) trackable.getListener());
                isVisible = listener.isVisible();
                telemetry.addData(trackable.getName(), isVisible ? "Visible" : "Not Visible");
                if (!isVisible) {
                    continue;
                }

                OpenGLMatrix lastLocation = listener.getUpdatedRobotLocation();
                if (lastLocation != null) {

                    RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                    telemetry.addData("Pos", format(lastLocation));

                    VectorF v = lastLocation.getTranslation();
                    double x = v.get(0);
                    double y = v.get(1);

                    Orientation ori = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double zAngle = ori.thirdAngle;

                    return new double[] {x, y, zAngle};
                }
            }
            telemetry.update();
        } catch (Exception e){
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            System.out.println(e.getMessage());
        }
        return null;
    }

    private void printRobotLocation(String tag, double[] loc) {
        if (loc != null)
            telemetry.addData(tag, "x = %.0f, y = %.0f, z = %.0f", loc[0], loc[1], loc[2]);
        else
            telemetry.addData(tag, "unknown");
        telemetry.update();
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
