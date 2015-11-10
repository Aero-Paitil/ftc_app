package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Aryoman on 11/8/2015.
 * This is the code for the mecanum wheels drive platform.
 */

public class MecanumWheelsOpMode extends OpMode {

    //defining the 4 motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;


    @Override
    public void init() {

        //"initializing" the motors
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorRearLeft = hardwareMap.dcMotor.get("RL");
        motorRearRight = hardwareMap.dcMotor.get("RR");

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        /*defining some movement for the robot. The left joystick right now controls the direction the wheels will spin,
        while the right joystick will make the robot actually turn instead of strafe.*/
        float forward = -gamepad1.left_stick_y;
        float right = -gamepad1.left_stick_x;
        float clockwise = -gamepad1.right_stick_y + gamepad1.right_stick_x; //Todo figure out how to translate x and y coordinates into angles

        //add deadband so you don't strafe when you don't want to. A deadband is essentially if you want to go to the right,
        //and the joystick is 7 degrees short of 90 degrees, instead of having the robot slowly creep forward, the robot will
        //ignore the small degrees and just go to the right.
        //todo adjust the deadband
        if ((right > -0.1) && (right < 0.1)) right = 0;


        float front_left = forward + clockwise + right;
        float front_right = forward - clockwise - right;
        float rear_left = forward + clockwise - right;
        float rear_right = forward - clockwise + right;

        /*this is scaling the motor power. Since our motors work on a scale between -1 and 1, and when we input
        values into the controller, they can be greater than one. We want to make sure that all values are between 1 and -1.
         we do that by first figuring out what the maximum value is, and then dividing all the numbers by the max value. Therefore
        the max power will be 1 (or -1, if we are going in reverse) and the other powers will be less than one. */
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

        /* assigning the motors the scaled powers that we just calculated in the step above. */
        motorFrontLeft.setPower(front_left);
        motorFrontRight.setPower(front_right);
        motorRearLeft.setPower(rear_left);
        motorRearRight.setPower(rear_right);

    }
}
