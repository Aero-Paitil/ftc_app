package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import javax.crypto.ExemptionMechanism;

/**
 * Created by Sangmin on 1/10/16.
 * Gyro calibration code - run init, wait for calibration to finish, point robot to right direction; start to record current heading
 */
public class GyroCalibration extends LinearOpMode {
    MecanumWheels mecanumWheels;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true);
        mecanumWheels.forwardSetup();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        sleep(500);
        double heading, drift=0;
        while (!opModeIsActive()) {
            waitOneFullHardwareCycle();
            telemetry.addData("Do not move the robot", "observe the drift");
            heading = mecanumWheels.getGyroHeading();
            drift = mecanumWheels.getHeadingError(0)/timer.time();
            telemetry.addData("heading", heading);
            telemetry.addData("drift", formatNum(60*drift)+" deg/min");
        }

        waitForStart();

        mecanumWheels.resetGyroHeading();
        timer.reset();
        while (opModeIsActive()) {
            waitOneFullHardwareCycle();
            heading = mecanumWheels.getGyroHeading();
            telemetry.addData("Point the robot, heading", heading);
            telemetry.addData("Estimated drift", formatNum(drift*timer.time())+" deg");
            MecanumWheels.pointedHeading = heading;
        }

    }

    /*
     format with 2 decimal points
     */
    private double formatNum(double num) {
        return ((int)(100*num))/100;
    }

}