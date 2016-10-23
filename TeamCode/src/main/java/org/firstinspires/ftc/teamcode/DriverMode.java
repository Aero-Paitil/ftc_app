package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by NBTeam on 10/23/2016.
 */

@TeleOp(name="Driver Mode", group="nb")
//@Disabled

public class DriverMode extends OpMode{

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
}
