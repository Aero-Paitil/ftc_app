package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

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

    enum State {initial, homeIn, homeInFolded, homeOutFolded, homeOut}

    State armPosition = State.initial;
    DcMotor shoulderMotor;
    Servo elbowServo, wristServo, twistServo;

    ElapsedTime time;


    public static class ArmPosition {
        //TODO: CHECK ALL POSITIONS
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

    public void moveArm(int position){
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//TODO how to set speed
    }


    public void dockArm() {
        switch (armPosition) {
            case initial:
            case homeIn:
            case homeInFolded:
            case homeOutFolded:
            case homeOut:
                //time.reset();
                break;

        }
    }

    public void undockArm() {

    }
}
