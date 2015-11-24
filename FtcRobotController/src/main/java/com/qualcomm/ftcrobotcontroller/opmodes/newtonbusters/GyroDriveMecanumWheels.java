package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;


/**
 * Created by Aryoman on 11/8/2015.
 * This is the code for the mecanum wheels drive platform.
 */

public class GyroDriveMecanumWheels extends OpMode {

    MecanumWheels mecanumWheels;


    @Override
    public void init() {

       mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.
    }

    @Override
    public void loop() {
        /*defining some movement for the robot. The left joystick right now controls the direction the wheels will spin,
        while the right joystick will make the robot actually turn instead of strafe.*/

        while(gamepad1.a && gamepad2.b){
           mecanumWheels.resetGyroHeading();
        }


        //We are using the joystick values in the driver's perspective (field coordinates).
        double fieldForward = -gamepad1.left_stick_y; //field coordinates
        double fieldRight = gamepad1.left_stick_x; //field coordinates
        double clockwise = gamepad1.right_stick_x; //doesn't matter field or robot
        mecanumWheels.powerMotors(fieldForward,fieldRight,clockwise,true);


    }
}
