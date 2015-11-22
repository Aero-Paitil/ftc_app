package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Allison 11/20/15
 * Modified by Jasmine and Simone 11/21/15
 * -added code to move brush handles and turn brushes
 */
public class BrushOpMode extends OpMode {
    Servo rightBrushHandle;
    Servo leftBrushHandle;
    Servo rightBrush;
    Servo leftBrush;

    @Override
    public void init() {
        rightBrushHandle = hardwareMap.servo.get("SM1");
        leftBrushHandle = hardwareMap.servo.get("SM2");
        rightBrush = hardwareMap.servo.get("Brush3");
        leftBrush = hardwareMap.servo.get("Brush4");

        rightBrush.setDirection(Servo.Direction.REVERSE);

        rightBrushHandle.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void loop() {
        // when a button is pressed, left trigger position controls the handle position
        // not pressed trigger means handle is out
        // pressed all the way trigger means handles in
        if (gamepad2.a){
            leftBrushHandle.setPosition(1-gamepad2.left_trigger);
            rightBrushHandle.setPosition(1-gamepad2.right_trigger);
        }


        //For continuous servos:
        //0=inward
        //1=outward
        //0.5=stop

        while (gamepad2.left_bumper){
            if (gamepad2.b) {
                leftBrush.setPosition(1);
            }
            else if(gamepad2.x){
                leftBrush.setPosition(0);
            }
            else if(gamepad2.y){
                leftBrush.setPosition(0.5);
            }
        }

        while (gamepad2.right_bumper){
            if (gamepad2.b) {
                rightBrush.setPosition(1);
            }
            else if(gamepad2.x){
                rightBrush.setPosition(0);
            }
            else if(gamepad2.y){
                rightBrush.setPosition(0.5);
            }
        }



    }
}