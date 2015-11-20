package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by tatianag on 11/20/15.
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

        rightBrush.setPosition(0);
        leftBrush.setPosition(0);

        // docked position is 0
        rightBrushHandle.setPosition(1); // all the way out
        leftBrushHandle.setPosition(0);  // all the way in


    }
}