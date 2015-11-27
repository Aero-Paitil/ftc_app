package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sangmin Lee on 11/27/15.
 */
public class ArmOpMode extends OpMode {

    Servo elbow1, box2, box3, extension4, extension5, beacon6, skiLiftHandleRight, skiLiftHandleLeft;
    Servo rightBrushHandle, leftBrushHandle, rightBrush, leftBrush;
    DcMotor arm;

    @Override
    public void init() {
        rightBrushHandle = hardwareMap.servo.get("BrushHandle1");
        leftBrushHandle = hardwareMap.servo.get("BrushHandle2");
        rightBrush = hardwareMap.servo.get("Brush3");
        leftBrush = hardwareMap.servo.get("Brush4");
        leftBrush.setPosition(0.5);
        rightBrush.setPosition(0.5);
        rightBrushHandle.setDirection(Servo.Direction.REVERSE);
        rightBrushHandle.setPosition(1);
        leftBrushHandle.setPosition(1);

        arm = hardwareMap.dcMotor.get("Arm");
        arm.setPower(0);

        elbow1 = hardwareMap.servo.get("Elbow1");
        elbow1.setPosition(0.4);
        telemetry.addData("elbow position", elbow1.getPosition());
        box2 = hardwareMap.servo.get("Box2");
        box2.setPosition(0.8); //0 position is flat against forearm
        telemetry.addData("box2 position", box2.getPosition());
        box3 = hardwareMap.servo.get("Box3");
        box3.setPosition(0.5);
        extension4 = hardwareMap.servo.get("Extension4");
        extension4.setPosition(1.0);
        extension5 = hardwareMap.servo.get("Extension5");
        extension5.setPosition(0.0);
        beacon6 = hardwareMap.servo.get("Beacon6");
        beacon6.setPosition(0.0);
        skiLiftHandleRight = hardwareMap.servo.get("SkiLiftHandle6");
        skiLiftHandleRight.setPosition(0.0);
        skiLiftHandleLeft = hardwareMap.servo.get("SkiLiftHandle5");
        skiLiftHandleLeft.setPosition(0.0);
    }

    @Override
    public void loop() {
        //elbow
        if (gamepad2.y) {
            double newposition = Range.clip(elbow1.getPosition() + 0.1, 0, 1);
            elbow1.setPosition(newposition);
            while(true){ //makes it so that you have to unpress before you can go again
                if (!gamepad2.y){
                    break;
                }
            }
        }else if (gamepad2.x){
            double newpostion = Range.clip(elbow1.getPosition() - 0.1, 0, 1);
            elbow1.setPosition(newpostion);
            while(true){
                if (!gamepad2.x){
                    break;
                }
            }
        }

        //box2
        if (gamepad2.b) {
            double newposition = Range.clip(box2.getPosition() + 0.1, 0, 1);
            box2.setPosition(newposition);
            while(true){
                if (!gamepad2.b){
                    break;
                }
            }
        }else if (gamepad2.a){
            double newpostion = Range.clip(box2.getPosition() - 0.1, 0, 1);
            box2.setPosition(newpostion);
            while(true){
                if (!gamepad2.a){
                    break;
                }
            }
        }

        //arm
        if (gamepad2.left_bumper) { //negative is back/up
            arm.setPower(0.5);
        }else if (gamepad2.right_bumper){
            arm.setPower(-0.5);
        }else{
            arm.setPower(-0.1);
        }
    }
}
