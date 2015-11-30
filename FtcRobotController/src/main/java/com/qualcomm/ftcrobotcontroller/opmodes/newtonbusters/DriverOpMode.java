package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Aryoman on 11/24/2015.
 */
public class DriverOpMode extends OpMode{
    final static private int REAR_WHEELS_COUNTS = 1000;

    MecanumWheels mecanumWheels;
    Brushes brushes;
    DcMotor rearWheels;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.
        brushes = new Brushes (hardwareMap ,telemetry);
        rearWheels = hardwareMap.dcMotor.get("RearWheels");
        rearWheels.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    @Override
    public void start(){
        rearWheels.setTargetPosition(-REAR_WHEELS_COUNTS);
        rearWheels.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rearWheels.setPower(0.3);
    }

    @Override
    public void loop() {

        while(gamepad1.a && gamepad1.b){
            mecanumWheels.resetGyroHeading();
        }

        //defining some movement for the robot. The left joystick right now controls the direction the wheels will spin,
        //while the right joystick will make the robot actually turn instead of strafe.

        //We are using the joystick values in the driver's perspective (field coordinates).
        double fieldForward = -gamepad1.left_stick_y; //field coordinates
        double fieldRight = gamepad1.left_stick_x; //field coordinates
        double clockwise = gamepad1.right_stick_x; //doesn't matter field or robot
        mecanumWheels.powerMotors(fieldForward, fieldRight, clockwise, true);


        //this code controls the brushes
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

        //this code controls the rear wheels
        telemetry.addData("rear wheel position", rearWheels.getCurrentPosition());
        if (gamepad1.a){
            rearWheels.setTargetPosition(rearWheels.getCurrentPosition() - 1000);
        }
        if (gamepad1.b){
            rearWheels.setTargetPosition(rearWheels.getCurrentPosition() + 1000);
        }
    }
}
