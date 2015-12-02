package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sangmin Lee on 11/27/15.
 * Modified by Athena Zheng on 11/29/15.
 */
public class ArmOpMode extends OpMode {

    static final double ELBOW_POS_UPDATE_TIME = 0.1;
    Servo extension4, extension5, beacon6, skiLiftHandleRight, skiLiftHandleLeft;
    Servo rightBrushHandle, leftBrushHandle, rightBrush, leftBrush;
    Arm arm;
    ElapsedTime elbowTime;

    @Override
    public void init() {
        elbowTime = new ElapsedTime();
        rightBrushHandle = hardwareMap.servo.get("BrushHandle1");
        leftBrushHandle = hardwareMap.servo.get("BrushHandle2");
        rightBrush = hardwareMap.servo.get("Brush3");
        leftBrush = hardwareMap.servo.get("Brush4");
        leftBrush.setPosition(0.5);
        rightBrush.setPosition(0.5);
        rightBrushHandle.setDirection(Servo.Direction.REVERSE);
        rightBrushHandle.setPosition(1);
        leftBrushHandle.setPosition(1);

        arm = new Arm(hardwareMap, telemetry);
        extension4 = hardwareMap.servo.get("Extension4");
        extension4.setPosition(1.0);
        extension5 = hardwareMap.servo.get("Extension5");
        extension5.setPosition(0.0);
        beacon6 = hardwareMap.servo.get("Beacon6");
        beacon6.setPosition(0.0);
        skiLiftHandleRight = hardwareMap.servo.get("SkiLiftHandle6");
        skiLiftHandleRight.setPosition(0.0);
        skiLiftHandleLeft = hardwareMap.servo.get("SkiLiftHandle5");
        skiLiftHandleLeft.setDirection(Servo.Direction.REVERSE);
        skiLiftHandleLeft.setPosition(0.0);
    }

    @Override
    public void start() {
        arm.holdShoulderPosition();

    }

    @Override
    public void loop() {
        arm.telemetry();


        //wrist
        if (gamepad2.y) {
            arm.moveWrist(0.01);
        /*
        while (true) {
            if (!gamepad2.b) {
                break;
            }
        }
        */
        } else if (gamepad2.a) {
            arm.moveWrist(-0.01);
        /*
        while (true) {
            if (!gamepad2.a) {
                break;
            }
        }
        */
        }
        //twist
        if (gamepad2.x) {
            arm.moveTwist(0.01);

        } else if (gamepad2.b) {
            arm.moveTwist(-0.01);

        }

        //shoulder
        if (gamepad2.left_stick_y != 0) { //negative is up
            arm.moveShoulder(gamepad2.left_stick_y);
        } else {
            arm.holdShoulderPosition();
        }

        //elbow
        if (gamepad2.right_stick_y != 0) { //negative is up
            if (elbowTime.time() > ELBOW_POS_UPDATE_TIME) {
                arm.moveElbow(-gamepad2.right_stick_y / 100);
                elbowTime.reset();
            }
        }

        if (gamepad2.dpad_up) {
            arm.undockArm();
        }
        if (gamepad2.dpad_down) {
            arm.dockArm();
        }
    }
}