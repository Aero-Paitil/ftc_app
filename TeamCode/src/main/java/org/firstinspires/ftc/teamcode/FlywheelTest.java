package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.text.DecimalFormat;

/**
 * testing flywheel stabilization behavior (speed vs time) with various power profiles
 */
@Autonomous(name = "Flywheel", group = "nb")
public class FlywheelTest extends LinearOpMode {

    private DcMotor motorFlywheel;
    private DcMotor motorBelt;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFlywheel = hardwareMap.dcMotor.get("GunRight");
        // run flywheels by speed
        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorBelt = hardwareMap.dcMotor.get("Belt");

        while (!isStarted()) {
            idle();
        }

        // if we hit stop after init, nothing else should happen
        if (!opModeIsActive()) {
            return;
        }

        testLoadedFlywheel(-0.68);
        testLoadedFlywheel(-0.69);
        testLoadedFlywheel(-0.7);
        testLoadedFlywheel(-0.71);
        testLoadedFlywheel(-0.72);

    }

    private void testLoadedFlywheel(double power) {

        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFlywheel.setPower(-0.5);
        sleep(1200);
        motorFlywheel.setPower(power);
        sleep(1200);
        //We think that the flywheel is stable now

        ElapsedTime timer = new ElapsedTime();
        StringBuffer sb = new StringBuffer("time,count\n");
        int count;
        double ms;
        timer.reset();
        int initCount = motorFlywheel.getCurrentPosition();

        motorBelt.setPower(1);
        DecimalFormat f = new DecimalFormat("0.##");
        while(timer.milliseconds() < 10000){
            ms = timer.milliseconds();
            count = Math.abs(motorFlywheel.getCurrentPosition() - initCount);
            sb.append(f.format(ms)).append(",").append(count).append("\n");
            sleep(20);
        }
        motorBelt.setPower(0);
        motorFlywheel.setPower(0);
        try {
            String powerStr = "" + (int) Math.round(power * 100);
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheelCounts1" + powerStr + ".txt");
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write(sb.toString());
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }
        sleep(15000);
    }

    private void testFlywheel(double power) {

        telemetry.addData("Test power", "" + power);
        telemetry.update();

        // the flywheels should move out in the opposite direction
        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int prevCount = motorFlywheel.getCurrentPosition();
        int count;

        StringBuffer sb = new StringBuffer("time,speed\n");

        motorFlywheel.setPower(-0.5);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double prevTimeMs = timer.milliseconds();
        double timeMs, timeDiff, speed;
        DecimalFormat f = new DecimalFormat("0.####");
        while (timer.milliseconds() < 1200) {
            timeMs = timer.milliseconds();
            timeDiff = timeMs - prevTimeMs;
            count = motorFlywheel.getCurrentPosition();
            speed = (Math.abs(count - prevCount) / timeDiff);
            sb.append(f.format(timeMs)).append(",").append(f.format(speed)).append("\n");
            prevTimeMs = timeMs;
            prevCount = count;
            sleep(150);
        }
        motorFlywheel.setPower(power);
        while (timer.milliseconds() < 10000) {
            timeMs = timer.milliseconds();
            timeDiff = timeMs - prevTimeMs;
            count = motorFlywheel.getCurrentPosition();
            speed = (Math.abs(count - prevCount) / timeDiff);
            sb.append(f.format(timeMs)).append(",").append(f.format(speed)).append("\n");
            prevTimeMs = timeMs;
            prevCount = count;
            sleep(150);
        }

        try {
            String powerStr = "" + (int) Math.round(power * 100);
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheel" + powerStr + ".txt");
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
