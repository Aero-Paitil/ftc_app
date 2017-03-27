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
import java.lang.Math;
import java.util.ArrayList;

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


        // test initial settling of the flywheel at different speeds
//        testSettling(-0.5);
//        testSettling(-0.6);
//        testSettling(-0.7);
//        testSettling(-0.8);

        // record counts vs ms when balls are passing through
//        testLoadedFlywheel(-0.66);
//        testLoadedFlywheel(-0.67);
//        testLoadedFlywheel(-0.68);
//        testLoadedFlywheel(-0.69);
//        testLoadedFlywheel(-0.70);

        // test flywheel settling with speed ramped to the desired
//        testRampedSettling(-0.6);
//        testRampedSettling(-0.67);

        // test flywheel when max power is applied first the switched to speed mode
        testMaxPowerSettling(675);
    }

    /**
     * test flywheel with max power - settling
     * @param targetV in counts/sec
     */
    private void testMaxPowerSettling(double targetV) {
        DecimalFormat f = new DecimalFormat("0.##"); // format for decimals
        StringBuffer sb = new StringBuffer("time,count,power\n"); // header for CSV file

        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // reset encoder counts, so that they start with 0
        motorFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // run flywheel in power mode
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // do not use brake, when 0 power is applied

        //start with small power, easier for motor than to start with full
        motorFlywheel.setPower(-0.2);
        sleep(100); //wait 100 milliseconds
        motorFlywheel.setPower(-1); //set full power

        ArrayList<Integer> counts = new ArrayList<>(21);
        ArrayList<Double> time = new ArrayList<>(21);
        int count; //encoder counts
        double ms; //milliseconds
        ElapsedTime t = new ElapsedTime();

        // initialize first 20 samples of both arraylists
        for (int n = 0; n <= 20; n++) {
            count = motorFlywheel.getCurrentPosition();
            ms = t.milliseconds();
            counts.add(count);

            time.add(ms);
            // save samples as a line to be written to a comma separated values file
            sb.append(f.format(ms)).append(",").append(count).append(",-1\n");
            sleep(10);
        }

        double v = 0; //velocity counts/sec

        // keep going until the velocity reaches target value
        while (v < targetV) {
            // calculate the velocity using the first and last sample
            v = 1000 * Math.abs(counts.get(20) - counts.get(0))/(time.get(20) - time.get(0));

            // delay for 10 milliseconds
            sleep(10);

            // save current encoder count and time in milliseconds into variables
            count = motorFlywheel.getCurrentPosition();
            ms = t.milliseconds();

            // delete the first element
            counts.remove(0);
            time.remove(0);

            // append last sample to the end of the list
            // at this point arrays have 21 samples with the last sample at the end
            counts.add(count);
            time.add(ms);

            // save current sample (encoder count and time)
            sb.append(f.format(ms)).append(",").append(count).append(",-1\n");
        }
        // set power to to 0
        //motorFlywheel.setPower(0);
//        telemetry.addData("target speed "+targetV+" reached ", f.format(ms)+" ms");
//        telemetry.update();

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // run flywheel in speed mode
        double endPower = -0.65; // the target power for speed mode

        // linear ramp
//        double settlingMs = 1000.0; //2000.0
//        double startPower = -0.63;
//        double power;
//        ElapsedTime timer = new ElapsedTime();
//        while(timer.milliseconds() < settlingMs){
//            // linear power up
//            power = startPower+timer.milliseconds()*(endPower-startPower)/settlingMs;
//            motorFlywheel.setPower(power);
//            ms = t.milliseconds();
//            count = motorFlywheel.getCurrentPosition();
//            sb.append(f.format(ms)).append(",").append(count).append(",").append(f.format(power)).append("\n");
//            sleep(10);
//        }

        // record sample just before transition
        ms = t.milliseconds();
        count = motorFlywheel.getCurrentPosition();
        sb.append(f.format(ms)).append(",").append(count).append(",").append(f.format(endPower)).append("\n");

        // set end speed (we are in speed mode - power means speed)
        motorFlywheel.setPower(endPower);

        // let the flywheel run at the target speed until 15 sec of testing are over
        while(t.milliseconds() < 15000){
            count = motorFlywheel.getCurrentPosition();
            ms = t.milliseconds();
            sb.append(f.format(ms)).append(",").append(count).append(",").append(f.format(endPower)).append("\n");
            sleep(10);
        }

        // stop flywheel
        motorFlywheel.setPower(0);

        // wait 10 sec for flywheel to stop
        sleep(10000);

        try {
            // save the data into file
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheelMaxPowerRampSettle" + targetV);
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write(sb.toString());
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }


        sleep(10000);

    }

    private void testSettling(double power) {
        DecimalFormat f = new DecimalFormat("0.##");
        ElapsedTime timer = new ElapsedTime();
        StringBuffer sb = new StringBuffer("time,count\n");

        telemetry.addData("testing ",power);
        telemetry.update();
        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFlywheel.setPower(-1);
        sleep(1200);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setPower(power);



        int count;
        double ms;
        timer.reset();
        int initCount = motorFlywheel.getCurrentPosition();;
        while(timer.milliseconds() < 10000){
            ms = timer.milliseconds();
            count = Math.abs(motorFlywheel.getCurrentPosition() - initCount);
            sb.append(f.format(ms)).append(",").append(count).append("\n");
            sleep(10);
        }
        // measure average speed counts/sec
        int c0 = motorFlywheel.getCurrentPosition();
        sleep(10000);
        int c1 = motorFlywheel.getCurrentPosition();
        telemetry.addData("maxSpeed ",motorFlywheel.getMaxSpeed());
        telemetry.addData("measured speed at "+power, Math.abs(c1-c0)/10.0);
        telemetry.update();
        motorFlywheel.setPower(0);
        try {
            String powerStr = "" + (int) Math.round(power * 100);
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheelSettle" + powerStr + ".txt");
            //telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write(sb.toString());
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }


        sleep(10000);

    }

    private void testRampedSettling(double power) {

        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double initPower = -0.2;
        motorFlywheel.setPower(initPower);

        DecimalFormat f = new DecimalFormat("0.##");
        ElapsedTime timer = new ElapsedTime();
        StringBuffer sb = new StringBuffer(64000).append("time,count\n");
        int count;
        double ms;
        timer.reset();
        int initCount = motorFlywheel.getCurrentPosition();
        double settlingMs = 2200.0;//2500.0; //2000.0
        while(timer.milliseconds() < settlingMs){
            ms = timer.milliseconds();
            count = Math.abs(motorFlywheel.getCurrentPosition() - initCount);
            //sb.append(f.format(ms)).append(",").append(count).append("\n");
            // linear power up
            motorFlywheel.setPower(initPower+ms*(power-initPower)/settlingMs);
            // sine power up from 0 pi to 1/2 pi
            //motorFlywheel.setPower(initPower + (power-initPower) * Math.sin(ms/settlingMs * Math.PI/2));
            // sine power up from 1/8 pi to 1/2 pi, but need to do multiply so the range still go from 0 to 1
            //double multi = 1/(1-Math.sin(Math.PI/8));
            //motorFlywheel.setPower(initPower + (power-initPower) * (Math.sin(Math.PI/8 + ms/settlingMs * (3*Math.PI/8))-Math.sin(Math.PI/8))*multi);
            //sigmoid curve - we will go from -1 to +6 range
            //double t= 7.0*ms / settlingMs - 1;
            //double newP = power / ( 1.0 + Math.exp(-t));
            //motorFlywheel.setPower(newP);
            //sb.append(f.format(ms)).append(",").append(newP).append(",").append(count).append("\n");
            sb.append(f.format(ms)).append(",").append(count).append("\n");
            sleep(10);
        }
        motorFlywheel.setPower(power);

        int c0 = motorFlywheel.getCurrentPosition();
        double t0 = timer.milliseconds();
        while(timer.milliseconds() < 10000){
            ms = timer.milliseconds();
            count = Math.abs(motorFlywheel.getCurrentPosition() - initCount);
            //sb.append(f.format(ms)).append(",").append(power).append(",").append(count).append("\n");
            sb.append(f.format(ms)).append(",").append(count).append("\n");
             sleep(10);
        }
        int c1 = motorFlywheel.getCurrentPosition();
        double t1 = timer.milliseconds();
        telemetry.addData("maxSpeed ",motorFlywheel.getMaxSpeed());
        telemetry.addData("measured speed at "+power, f.format(1000*Math.abs(c1-c0)/(t1-t0)));

        motorFlywheel.setPower(0);
        // measure average speed counts/sec
        try {
            String powerStr = "" + (int) Math.round(power * 100);
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheelLinear2200msRamp" + powerStr);
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write(sb.toString());
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }
        telemetry.update();
        sleep(10000);
    }


    private void testLoadedFlywheel(double power) {

        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFlywheel.setPower(-0.3);
        sleep(500);
        motorFlywheel.setPower(-0.5);
        sleep(400);
        motorFlywheel.setPower(-0.6);
        sleep(300);
        motorFlywheel.setPower(power);
        sleep(15000);
        //We think that the flywheel is stable now

        ElapsedTime timer = new ElapsedTime();
        StringBuffer sb = new StringBuffer("time,count\n");
        int count;
        double ms;
        timer.reset();
        int initCount = motorFlywheel.getCurrentPosition();

        motorBelt.setPower(1);
        DecimalFormat f = new DecimalFormat("0.##");
        while(timer.milliseconds() < 7000){
            ms = timer.milliseconds();
            count = Math.abs(motorFlywheel.getCurrentPosition() - initCount);
            sb.append(f.format(ms)).append(",").append(count).append("\n");
            sleep(10);
        }
        motorBelt.setPower(0);
        motorFlywheel.setPower(0);
        try {
            String powerStr = "" + (int) Math.round(power * 100);
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheelCounts2withDelay" + powerStr);
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write(sb.toString());
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }
        sleep(7000);
    }


    private void testLoadedFlywheel1(double power) {

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
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheelCounts1" + powerStr);
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

        ElapsedTime timer = new ElapsedTime();
        double prevTimeMs = timer.milliseconds();
        double timeMs, timeDiff, speed;
        DecimalFormat f = new DecimalFormat("0.####");
        timer.reset();
        motorFlywheel.setPower(-0.5);
        while (timer.milliseconds() < 500) {
            timeMs = timer.milliseconds();
            timeDiff = timeMs - prevTimeMs;
            count = motorFlywheel.getCurrentPosition();
            speed = (Math.abs(count - prevCount) / timeDiff);
            sb.append(f.format(timeMs)).append(",").append(f.format(speed)).append("\n");
            prevTimeMs = timeMs;
            prevCount = count;
            sleep(250);
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
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/flywheel" + powerStr);
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
