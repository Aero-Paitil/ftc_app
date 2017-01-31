package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;

/**
 * testing flywheel stabilization behavior (speed vs time) with various power profiles
 */
@Autonomous(name="Flywheel", group="nb")
public class FlywheelTest extends LinearOpMode {

    private DcMotor motorFlywheel;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFlywheel = hardwareMap.dcMotor.get("GunRight");
        // run flywheels by speed
        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!isStarted()) {
            idle();
        }

        // if we hit stop after init, nothing else should happen
        if (!opModeIsActive()) {
            return;
        }

        testFlywheel(-0.5);
        testFlywheel(-0.6);
        testFlywheel(-0.7);
        testFlywheel(-0.8);
        testFlywheel(-0.9);

    }

    private void testFlywheel(double power) {

        telemetry.addData("Test power", ""+power);
        telemetry.update();

        // the flywheels should move out in the opposite direction
        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int prevCount = motorFlywheel.getCurrentPosition();
        int count;

        StringBuffer sb = new StringBuffer("time,speed\n");

        motorFlywheel.setPower(power);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double prevTimeMs = timer.milliseconds();
        double timeMs, timeDiff, speed;
        while (timer.milliseconds() < 7000) {
            timeMs = timer.milliseconds();
            timeDiff = timeMs - prevTimeMs;
                count = motorFlywheel.getCurrentPosition();
                speed = (Math.abs(count - prevCount) / timeDiff);
                sb.append(timeMs).append(",").append(speed).append("\n");
                prevTimeMs = timeMs;
                prevCount = count;
            sleep(70);
        }

        try {
            String powerStr = ""+(int)Math.round(power*100);
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheel"+powerStr+".txt");
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write(sb.toString());
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }

        motorFlywheel.setPower(0);
        sleep(10000);

    }

}
