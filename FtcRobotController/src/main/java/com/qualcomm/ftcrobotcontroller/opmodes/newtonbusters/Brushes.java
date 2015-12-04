package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Aryoman on 11/24/2015.
 */
public class Brushes {
    static final double TASK_TIME = 0.5;
    static final double UNDOCKED_POSITION = 0.9;
    static final double RIGHT_DOCKED_POSITION = 0.1;
    static final double LEFT_DOCKED_POSITION = 0.3;
    static final double ROTATE_INWARD_SLOW = 0.1;
    static final double ROTATE_OUTWARD_SLOW = 0.9;
    public static final double ROTATE_INWARD_FAST = 0;
    public static final double ROTATE_OUTWARD_FAST = 1;
    public static final double STOP_ROTATION = 0.5;

    Servo rightBrushHandle;
    Servo leftBrushHandle;
    Servo rightBrush;
    Servo leftBrush;

    ElapsedTime time;

    enum State {brushesUndocked, brushesSpread, rightBrushDocking, leftBrushDocking, brushesDocked, leftBrushUndocking, rightBrushUndocking}

    State brushState;

    Telemetry telemetry;

    public Brushes(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        rightBrushHandle = hardwareMap.servo.get("BrushHandle1");
        leftBrushHandle = hardwareMap.servo.get("BrushHandle2");
        rightBrush = hardwareMap.servo.get("Brush3");
        leftBrush = hardwareMap.servo.get("Brush4");

        rightBrush.setDirection(Servo.Direction.REVERSE);
        rightBrushHandle.setDirection(Servo.Direction.REVERSE);

        time = new ElapsedTime();
        brushState = State.brushesDocked;
        leftBrush.setPosition(STOP_ROTATION);
        rightBrush.setPosition(STOP_ROTATION);
        leftBrushHandle.setPosition(LEFT_DOCKED_POSITION);
        rightBrushHandle.setPosition(RIGHT_DOCKED_POSITION);
    }

    /*
    undock brushes: left, then right
     */
    public void undockBrushes() {

        double currenttime = time.time();
        telemetry.addData("Current Time", currenttime);
        switch (brushState) {
            case brushesDocked:
                brushState = State.leftBrushUndocking;
                time.reset();
                break;
            case leftBrushUndocking:
                leftBrushHandle.setPosition(1);
                leftBrush.setPosition(ROTATE_INWARD_SLOW);
                if (currenttime > TASK_TIME) {
                    leftBrush.setPosition(STOP_ROTATION);
                    brushState = State.rightBrushUndocking;
                    time.reset();
                }
                break;
            case rightBrushUndocking:
                rightBrushHandle.setPosition(1);
                rightBrush.setPosition(ROTATE_INWARD_SLOW);
                if (currenttime > TASK_TIME) {
                    rightBrush.setPosition(STOP_ROTATION);
                    brushState = State.brushesSpread;
                    time.reset();
                }
                break;
            case brushesSpread:
                leftBrushHandle.setPosition(UNDOCKED_POSITION-0.1);
                rightBrushHandle.setPosition(UNDOCKED_POSITION);
                if (currenttime > TASK_TIME) {
                    brushState = State.brushesUndocked;
                    time.reset();
                }
                break;
            case brushesUndocked:
                time.reset();
                break;
            case rightBrushDocking:
                brushState = State.rightBrushUndocking;
                time.reset();
                break;
            case leftBrushDocking:
                brushState = State.rightBrushUndocking;
                time.reset();
                break;
        }

    }

    public void autonomousUndockBrushes()
    {
        leftBrushHandle.setPosition(1);
        leftBrush.setPosition(ROTATE_INWARD_SLOW);
        Utils.delay(TASK_TIME);
        rightBrushHandle.setPosition(1);
        rightBrush.setPosition(ROTATE_INWARD_SLOW);
        Utils.delay(TASK_TIME);
        leftBrush.setPosition(STOP_ROTATION);
        rightBrush.setPosition(STOP_ROTATION);
        leftBrush.setPosition(UNDOCKED_POSITION-0.1);
        rightBrush.setPosition(UNDOCKED_POSITION);

    }



    /*
    dock brushes: right, then left
     */
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
                brushState = State.rightBrushDocking;
                time.reset();
                break;
            case rightBrushDocking:
                rightBrushHandle.setPosition(RIGHT_DOCKED_POSITION);
                rightBrush.setPosition(ROTATE_OUTWARD_SLOW);
                if (currenttime > TASK_TIME) {
                    rightBrush.setPosition(STOP_ROTATION);
                    brushState = State.leftBrushDocking;
                    time.reset();
                }
                break;
            case leftBrushUndocking:
            case leftBrushDocking:
                leftBrushHandle.setPosition(LEFT_DOCKED_POSITION);
                leftBrush.setPosition(ROTATE_OUTWARD_SLOW);
                if (currenttime > TASK_TIME) {
                    leftBrush.setPosition(STOP_ROTATION);
                    brushState = State.brushesDocked;
                    time.reset();
                }
                break;
            case brushesDocked:
                //rightBrush.setPosition(STOP_ROTATION);
                //leftBrush.setPosition(STOP_ROTATION);
                time.reset();
                break;
        }
    }

    public State getState() {
        return brushState;
    }

    public void printTelemetry() {
        telemetry.addData("Brush State", brushState);
        telemetry.addData("Left,Right Handle Pos", rightBrushHandle.getPosition() + ", " + leftBrushHandle.getPosition());
    }

    public void setRotation(double rotationPosition) {
        rightBrush.setPosition(rotationPosition);
        leftBrush.setPosition(rotationPosition);
    }
}
