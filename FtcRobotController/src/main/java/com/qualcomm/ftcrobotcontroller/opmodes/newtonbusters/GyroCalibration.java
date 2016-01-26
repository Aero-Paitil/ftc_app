package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import javax.crypto.ExemptionMechanism;

/**
 * Created by Sangmin on 1/10/16.
 */
public class GyroCalibration extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumWheels mecanumWheels;
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true);
        mecanumWheels.forwardSetup();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("GyroHeading", mecanumWheels.getGyroHeading());
            waitOneFullHardwareCycle();
        }
    }
}