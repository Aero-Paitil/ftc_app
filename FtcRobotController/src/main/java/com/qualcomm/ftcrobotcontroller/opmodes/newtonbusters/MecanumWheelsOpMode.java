package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Aryoman on 11/8/2015.
 * This is the code for the mecanum wheels drive platform.
 */

public class MecanumWheelsOpMode extends OpMode {

    MecanumWheels mecanumWheels;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, false); //we are not using Gyro.
    }

    @Override
    public void loop() {
        /*defining some movement for the robot. The left joystick right now controls the direction the wheels will spin,
        while the right joystick will make the robot actually turn instead of strafe.*/
        float forward = -gamepad1.left_stick_y;
        float right = gamepad1.left_stick_x;
        float clockwise = gamepad1.right_stick_x;

        mecanumWheels.powerMotors(forward, right, clockwise);
    }
}
