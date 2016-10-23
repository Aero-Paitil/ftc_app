package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by NBTeam on 10/23/2016.
 */

@TeleOp(name="Driver Mode", group="nb")
//@Disabled

public class DriverMode extends OpMode{

    public static final String TAG = "Vuforia Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    //defining the 4 motors
    DcMotor motorLeft1;
    DcMotor motorRight1;
    DcMotor motorLeft2;
    DcMotor motorRight2;

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

        // The following code is the VuforiaTest code up to the while loop that checks opModeIsActive().
        // ---- START VUFORIA CODE ----
        try{
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            SharedPreferences prefs = getSharedPrefs(hardwareMap);

            String licensekey = prefs.getString("vuforiaLicenseKey", null);

            parameters.vuforiaLicenseKey = licensekey;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            VuforiaTrackables allImages = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
            List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>(allImages);

            VuforiaTrackable wheelTarget = allImages.get(0);
                wheelTarget.setName("WheelTarget");
            VuforiaTrackable toolsTarget = allImages.get(1);
                wheelTarget.setName("ToolsTarget");
            VuforiaTrackable legosTarget = allImages.get(2);
                wheelTarget.setName("LegosTarget");
            VuforiaTrackable gearsTarget = allImages.get(3);
                wheelTarget.setName("GearsTarget");



        }catch (Exception e) {
            System.out.print(e.getMessage());
            return;
        }
        // ---- END VUFORIA CODE ----
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
            } else if (gamepad1.dpad_right) {
                leftForward = dpadSpeed;
                rightForward = -dpadSpeed;
            }
        }

        /* assigning the motors the scaled powers that we just calculated in the step above. */
        motorLeft1.setPower(leftForward);
        motorRight1.setPower(rightForward);
        motorLeft1.setPower(leftForward);
        motorRight2.setPower(rightForward);


    }


    String format(OpenGLMatrix transformationMatrix) {
        VectorF v = transformationMatrix.getTranslation();
        return ("{" + v.get(0) + ", " + v.get(1) + ", " + v.get(2) + "}");
        //return transformationMatrix.formatAsTransform();
    }

    public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }

}
