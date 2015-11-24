package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Aryoman on 11/24/2015.
 */
public class DriverOpMode extends OpMode{
    MecanumWheels mecanumWheels;
    Brushes brushes;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.
        brushes = new Brushes (hardwareMap ,telemetry);
    }

    @Override
    public void loop() {

        while(gamepad1.a && gamepad2.b){
            mecanumWheels.resetGyroHeading();
        }

        //defining some movement for the robot. The left joystick right now controls the direction the wheels will spin,
        //while the right joystick will make the robot actually turn instead of strafe.

        //We are using the joystick values in the driver's perspective (field coordinates).
        double fieldForward = -gamepad1.left_stick_y; //field coordinates
        double fieldRight = gamepad1.left_stick_x; //field coordinates
        double clockwise = gamepad1.right_stick_x; //doesn't matter field or robot
        mecanumWheels.powerMotors(fieldForward,fieldRight,clockwise,true);

        brushes.printTelemetry();

        if (gamepad2.dpad_up) {
            brushes.undockBrushes();
        } else if (gamepad2.dpad_down) {
            brushes.dockBrushes();
        } else if (gamepad2.dpad_right) {
            //rotate brushes inward
            brushes.setRotation(Brushes.ROTATE_INWARD_FAST);
        } else if (gamepad2.dpad_left) {
            //rotate brushes outward
            brushes.setRotation(Brushes.ROTATE_OUTWARD_FAST);
        } else if (gamepad2.x) {
            //stop the movement of the brushes
            brushes.setRotation(Brushes.STOP_ROTATION);
        }

    }
}
