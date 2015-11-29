package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jasmine on 11/28/15.
 * converting from hardware values to software values-
 * SERVOS: hardware values: 0-255
 * SERVOS: software values: 0-1
 * OPTION COMMAND L TO REFORMAT
 */
public class Arm {

    static final double TASK_TIME = 0.5;

    boolean movingToPosition = false;

    enum State {initial, homeIn, homeInFolded, homeOutFolded, homeOut}

    State armPosition = State.initial;
    DcMotor shoulderMotor;
    Servo elbowServo, wristServo, twistServo;

    ElapsedTime time;


    public static class ArmPosition {
        static ArmPosition DRIVER_LOW_LIMIT = new ArmPosition(-550, 0/255, 0);
        static ArmPosition DRIVER_HIGH_LIMIT = new ArmPosition(-903, 195/255, 1);
        //TODO: CHECK ALL POSITIONS!!!
        static ArmPosition INITIAL = new ArmPosition(0, 115 / 255, 180 / 255);
        static ArmPosition HOME_IN = new ArmPosition(-42, 122 / 255, 180 / 255);
        static ArmPosition HOME_IN_FOLDED = new ArmPosition(76, 112 / 255, 255 / 255);
        static ArmPosition HOME_OUT_FOLDED = new ArmPosition(-550, 112 / 255, 255 / 255);
        static ArmPosition HOME_OUT = new ArmPosition(-550, 125 / 255, 255 / 255);

        /**
         * armPosition is defined by encoder counts
         * if 0 is initial position, box is all the way back and on top of color sensor
         * number of counts per rotation is 1140 for AndyMark DC Motor
         */
        int shoulder;
        //elbowPosition and wristPosition are controlled by servos
        //limits are 0 to 1
        double elbow;
        double wrist;

        public ArmPosition(int armPosition, double elbowPosition, double wristPosition) {
            this.shoulder = armPosition;
            this.elbow = elbowPosition;
            this.wrist = wristPosition;
        }
    }

    /**
     * transition between HOME_OUT to HOME_IN
     */

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        time = new ElapsedTime();

        shoulderMotor = hardwareMap.dcMotor.get("ArmPosition");
        shoulderMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        elbowServo = hardwareMap.servo.get("Elbow1");
        wristServo = hardwareMap.servo.get("Box2");
        twistServo = hardwareMap.servo.get("Box3");


    }

    public void start() {
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        shoulderMotor.setPower(-0.1);
    }

    private void moveArmLoop(int position) {
        double currentPosition = shoulderMotor.getCurrentPosition();
        if (currentPosition == position) {
            return;
        } else if (!movingToPosition) {
            shoulderMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            double power;
            if (position - currentPosition < 0) {
                power = -0.1;
            } else {
                power = 0.1;
            }
            shoulderMotor.setPower(power);
            movingToPosition = true;
        } else {
            double acceptableDifference = 10;
            double currentDifference = Math.abs(position - currentPosition);
            if (currentDifference <= acceptableDifference) {
                shoulderMotor.setTargetPosition(position);
                shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                while (currentPosition != position) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        DbgLog.logStacktrace(e);
                    }
                }
                movingToPosition = false;
            }

        }


//TODO how to set speed
    }

    /**
     * while speed is not 0, move arm
     * @param speed is from -1 to 1
     */
    public void moveArm(double speed){
        if (Math.abs(speed) < 0.1){
            int currentPosition = shoulderMotor.getCurrentPosition();
            shoulderMotor.setTargetPosition(currentPosition);
            shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        shoulderMotor.setPower(speed/10);
    }

    public void moveElbow(){

    }

    public void dockArm() {
        if (armPosition == State.homeIn) {
            return;
        }
        ArmPosition p = null;
        State ns = null; //next state
        double currenttime = time.time();
        switch (armPosition) {
            case homeInFolded:
            case initial:
                p = ArmPosition.HOME_IN;
                ns = State.homeIn;
                break;
            case homeIn:
                //time.reset();
                break;
            case homeOutFolded:
                p = ArmPosition.HOME_IN_FOLDED;
                ns = State.homeInFolded;
                break;
            case homeOut:
                p = ArmPosition.HOME_OUT_FOLDED;
                ns = State.homeOutFolded;
                break;
        }
        if (p != null && ns != null) {
            moveArmLoop(p.shoulder);
            elbowServo.setPosition(p.elbow);
            wristServo.setPosition(p.wrist);
            if (currenttime > TASK_TIME) {
                armPosition = ns;
                time.reset();
            }
        }
    }

    public void undockArm() {
        if (armPosition == State.homeOut) {
            return;
        }
        ArmPosition p = null;
        State ns = null; //next state
        double currenttime = time.time();
        switch (armPosition) {
            case initial:
            case homeIn:
                p = ArmPosition.HOME_IN_FOLDED;
                ns = State.homeInFolded;
                break;
            case homeInFolded:
                p = ArmPosition.HOME_OUT_FOLDED;
                ns = State.homeOutFolded;
                break;
            case homeOutFolded:
                p = ArmPosition.HOME_OUT;
                ns = State.homeOut;
                break;
        }
        if (p != null && ns != null) {
            moveArmLoop(p.shoulder);
            elbowServo.setPosition(p.elbow);
            wristServo.setPosition(p.wrist);
            if (currenttime > TASK_TIME) {
                armPosition = ns;
                time.reset();
            }
        }
    }
}
