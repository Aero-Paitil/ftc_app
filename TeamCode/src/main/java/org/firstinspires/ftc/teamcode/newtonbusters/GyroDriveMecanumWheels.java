package org.firstinspires.ftc.teamcode.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;


/**
 * Created by Aryoman on 11/8/2015.
 * This is the code for the mecanum wheels drive platform.
 */

public class GyroDriveMecanumWheels extends OpMode {

    MecanumWheels mecanumWheels;
    DcMotor rearWheels;


    @Override
    public void init() {

        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.
        mecanumWheels.forwardSetup();
        rearWheels = hardwareMap.dcMotor.get("RearWheels");
        rearWheels.setMode(DcMotor.RunMode.RESET_ENCODERS);

    }

    @Override
    public void start() {

        //rearWheels.setTargetPosition(-REAR_WHEELS_COUNTS);
        rearWheels.setTargetPosition(0);
        rearWheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearWheels.setPower(0.3);
    }


    @Override
    public void loop() {
        //changing field coordinates
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            mecanumWheels.resetGyroHeading();
            while (true) {
                if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    break;
                }
            }
        }

        double clockwise = gamepad1.right_stick_x; //doesn't matter field or robot
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            //We are using robot coordinates
            double forward = 0;
            double right = 0;
            if (gamepad1.dpad_up) {
                forward = 1;
            } else if (gamepad1.dpad_down) {
                forward = -1;
            }
            if (gamepad1.dpad_right) {
                right = 1;
            } else if (gamepad1.dpad_left) {
                right = -1;
            }
            mecanumWheels.powerMotors(forward, right, clockwise, false);//Not using gyro
        } else {
            //We are using the joystick values in the driver's perspective (field coordinates).
            double fieldForward = -gamepad1.left_stick_y; //field coordinates
            double fieldRight = gamepad1.left_stick_x; //field coordinates
            mecanumWheels.powerMotors(fieldForward, fieldRight, clockwise, true);
        }

        //this code controls the rear wheels
        //todo check limits
        telemetry.addData("rear wheel position", rearWheels.getCurrentPosition());
        if (gamepad1.a) {
            rearWheels.setTargetPosition(rearWheels.getCurrentPosition() + 500);
        }
        if (gamepad1.b) {
            rearWheels.setTargetPosition(rearWheels.getCurrentPosition() - 500);
        }


    }
}
