package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Allison 11/20/15
 * Modified by Jasmine and Simone 11/21/15
 * -added code to move brush handles and turn brushes
 *
 * !!BRUSHES SHOULD BE DOCKED RIGHT FIRST THEN LEFT!!
 * To complete action, hold down dpad until aciton is complete
 */



public class BrushOpMode extends OpMode {
    static final double TASK_TIME = 1;
    static final double UNDOCKED_POSITION = 0.9;

    Servo rightBrushHandle;
    Servo leftBrushHandle;
    Servo rightBrush;
    Servo leftBrush;

    ElapsedTime time;
    enum State{brushesUndocked, brushesSpread, rightBrushDocking, leftBrushDocking, brushesDocked, leftBrushUndocking, rightBrushUndocking}
    State brushState;

    @Override
        public void init() {
            rightBrushHandle = hardwareMap.servo.get("SM1");
            leftBrushHandle = hardwareMap.servo.get("SM2");
            rightBrush = hardwareMap.servo.get("Brush3");
            leftBrush = hardwareMap.servo.get("Brush4");

            rightBrush.setDirection(Servo.Direction.REVERSE);

            rightBrushHandle.setDirection(Servo.Direction.REVERSE);

            time = new ElapsedTime();
            brushState = State.brushesDocked;

        }

        //brushesUndocked, brushesSpread, rightBrushDocking, leftBrushDocking, brushesDocked, leftBrushUndocking, rightBrushUndocking};
        public void undockBrushes(){

            double currenttime = time.time();
            switch(brushState){
                case brushesUndocked:
                    time.reset();
                    break;
                case brushesSpread:
                    leftBrushHandle.setPosition(UNDOCKED_POSITION);
                    rightBrushHandle.setPosition(UNDOCKED_POSITION);
                    if (currenttime > TASK_TIME){
                        brushState = State.brushesUndocked;
                        time.reset();
                    }
                    break;
                case rightBrushDocking:
                    rightBrushHandle.setPosition(1);
                    if (currenttime > TASK_TIME){
                        brushState = State.rightBrushUndocking;
                        time.reset();
                    }
                    break;
                case leftBrushDocking:
                    leftBrushHandle.setPosition(1);
                    if (currenttime > TASK_TIME){
                        brushState = State.rightBrushUndocking;
                        time.reset();
                    }
                    break;
                case leftBrushUndocking:
                case brushesDocked:
                    leftBrushHandle.setPosition(1);
                    if (currenttime > TASK_TIME){
                        brushState = State.leftBrushUndocking;
                        time.reset();
                    }
                    break;
                case rightBrushUndocking:
                    rightBrushHandle.setPosition(1);
                    if (currenttime > TASK_TIME){
                        brushState = State.brushesSpread;
                        time.reset();
                    }
                    break;
        }

        }

    public void dockBrushes() {

        double currenttime = time.time();
        switch (brushState) {
            case brushesUndocked:
                leftBrushHandle.setPosition(1);
                rightBrushHandle.setPosition(1);
                if (currenttime > TASK_TIME) {
                    brushState = State.brushesSpread;
                    time.reset();
                }
                break;
            case brushesSpread:
            case rightBrushUndocking:
                rightBrushHandle.setPosition(0);
                if (currenttime > TASK_TIME) {
                    brushState = State.rightBrushDocking;
                    time.reset();
                }
                break;
            case rightBrushDocking:
                rightBrushHandle.setPosition(0);
                if (currenttime > TASK_TIME) {
                    brushState = State.leftBrushDocking;
                    time.reset();
                }
                break;
            case leftBrushUndocking:
            case leftBrushDocking:
                leftBrushHandle.setPosition(0);
                if (currenttime > TASK_TIME) {
                    brushState = State.brushesDocked;
                    time.reset();
                }
                break;
            case brushesDocked:
                rightBrush.setPosition(0.5);
                leftBrush.setPosition(0.5);
                time.reset();
                break;
        }
    }

    @Override
    public void loop() {
        telemetry.addData("Brush State", brushState);
        telemetry.addData("Left Position", leftBrushHandle.getPosition());
        telemetry.addData("Right Position", rightBrushHandle.getPosition());

        if (gamepad2.dpad_up){
            undockBrushes();
        }
        else if (gamepad2.dpad_down){
            dockBrushes();
        }

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