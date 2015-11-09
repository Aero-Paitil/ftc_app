package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Aryoman on 11/8/2015.
 * This is the code for the mecanum wheels drive platform.
 */

public class MecanumWheelsOpMode extends OpMode {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorRearLeft = hardwareMap.dcMotor.get("RL");
        motorRearRight = hardwareMap.dcMotor.get("RR");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        float forward = -gamepad1.left_stick_y;
        float right = -gamepad1.left_stick_x;
        float clockwise = -gamepad1.right_stick_y + gamepad1.right_stick_x; //Todo figure out how to translate x and y coordinates into angles

        //add deadband so you don't strafe when you don't want to
        //todo adjust the deadband
        if ((right > -0.1) && (right < 0.1)) right = 0;


        float front_left = forward + clockwise + right;
        float front_right = forward - clockwise - right;
        float rear_left = forward + clockwise - right;
        float rear_right = forward - clockwise + right;

        //todo find the maximum absolute value for any motor scaled power
        float max = Math.abs(front_left);
        max = Math.max(Math.abs(front_right), max);
        max = Math.max(Math.abs(rear_left), max);
        max = Math.max(Math.abs(rear_right), max);

        if (max > 1) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }
        motorFrontLeft.setPower(front_left);
        motorFrontRight.setPower(front_right);
        motorRearLeft.setPower(rear_left);
        motorRearRight.setPower(rear_right);

    }
}
