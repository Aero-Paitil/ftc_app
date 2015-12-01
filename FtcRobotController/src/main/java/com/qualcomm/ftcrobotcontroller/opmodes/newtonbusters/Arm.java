package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jasmine on 11/28/15.
 * converting from hardware values to software values-
 * SERVOS: hardware values: 0-255
 * SERVOS: software values: 0-1
 * OPTION COMMAND L TO REFORMAT
 */
public class Arm {

    static final double TASK_TIME = 1.0;
    static final double HOLD_POSITION_POWER = 0.2;


    enum State {initial, toHomeIn, homeIn, toHomeInFolded, toHomeOutFolded, toHomeOut, homeOut}

    State armState;
    DcMotor shoulderMotor;
    Servo elbowServo, wristServo, twistServo;

    ElapsedTime time;
    Telemetry telemetry;


    public static class ArmPosition {
        //static ArmPosition DRIVER_LOW_LIMIT = new ArmPosition(-550, 0/255, 0);
        //static ArmPosition DRIVER_HIGH_LIMIT = new ArmPosition(-903, 195.0/255, 1);
        //TODO: CHECK ALL POSITIONS!!!
        static ArmPosition INITIAL = new ArmPosition(0, 105.0/255, 1);
        static ArmPosition HOME_IN_FINAL = new ArmPosition(-160, 120.0/255, 185.0/255);
        static ArmPosition HOME_IN = new ArmPosition(-170, 120.0/255, 1); // wrist is adjusted in the end
        static ArmPosition HOME_IN_FOLDED = new ArmPosition(-130, 110.0/255, 1);
        static ArmPosition HOME_OUT_FOLDED = new ArmPosition(-580, 110.0/255, 1);
        static ArmPosition HOME_OUT = new ArmPosition(-580, 125.0/255, 1);

        /**
         * armState is defined by encoder counts
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
        this.telemetry = telemetry;
        shoulderMotor = hardwareMap.dcMotor.get("Arm");
        shoulderMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armState = State.initial;
        ArmPosition initialPosition = ArmPosition.INITIAL;
        elbowServo = hardwareMap.servo.get("Elbow1");
        elbowServo.setPosition(initialPosition.elbow);
        wristServo = hardwareMap.servo.get("Box2");
        wristServo.setPosition(initialPosition.wrist);
        twistServo = hardwareMap.servo.get("Box3");
        twistServo.setPosition(0.45);
        telemetry();
    }


    public void telemetry() {
        telemetry.addData("Shoulder", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow", elbowServo.getPosition());
        telemetry.addData("Wrist", wristServo.getPosition());
        telemetry.addData("Twist", twistServo.getPosition());
        telemetry.addData("Arm State", armState);
    }

    private void setShoulderPosition(int position) {
        shoulderMotor.setTargetPosition(position);
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        shoulderMotor.setPower(HOLD_POSITION_POWER);
    }

    public void holdShoulderPosition() {
        int currentPosition = shoulderMotor.getCurrentPosition();
        setShoulderPosition(currentPosition);
    }

    /**
     * while speed is not 0, move arm
     * @param speed is from -1 to 1
     */
    public void moveShoulder(double speed){
        if (Math.abs(speed) < 0.1){
            holdShoulderPosition();
        }
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        shoulderMotor.setPower(speed / 10);
    }

    public void moveElbow(double positionChange){
        double newPosition = elbowServo.getPosition() + positionChange;
        elbowServo.setPosition(Range.clip(newPosition, 0, 1));
    }

    public void moveWrist(double positionChange) {
        double newPosition = wristServo.getPosition() + positionChange;
        wristServo.setPosition(Range.clip(newPosition, 0, 1));
    }

    private void setArmPosition(ArmPosition p, State followingState, double currenttime) {
        wristServo.setPosition(p.wrist);
        elbowServo.setPosition(p.elbow);
        setShoulderPosition(p.shoulder);

        if (currenttime > TASK_TIME) {
            armState = followingState;
            time.reset();
        }
    }

    //initial, toHomeIn, homeIn, toHomeInFolded, toHomeOutFolded, toHomeOut, homeOut

    public void dockArm() {
        if (armState == State.homeIn) {
            setArmPosition(ArmPosition.HOME_IN_FINAL, State.homeIn, 0);
            return;
        }
        ArmPosition p = null;
        State followingState = null;
        double currenttime = time.time();
        switch (armState) {
            case initial:
            case homeOut:
            case toHomeOut:
                armState = State.toHomeOutFolded;
                time.reset();
                break;
            case toHomeOutFolded:
                p = ArmPosition.HOME_OUT_FOLDED;
                followingState = State.toHomeInFolded;
                break;
            case toHomeInFolded:
                p = ArmPosition.HOME_IN_FOLDED;
                followingState = State.toHomeIn;
                break;
            case toHomeIn:
                p = ArmPosition.HOME_IN;
                followingState = State.homeIn;
                break;
        }
        if (p != null) {
            setArmPosition(p, followingState, currenttime);
        }
    }

    public void undockArm() {
        if (armState == State.homeOut) {
            return;
        }
        ArmPosition p = null;
        State followingState = null;
        double currenttime = time.time();
        switch (armState) {
            case toHomeIn:
            case homeIn:
                armState = State.toHomeInFolded;
                time.reset();
                break;
            case toHomeInFolded:
                p = ArmPosition.HOME_IN_FOLDED;
                followingState = State.toHomeOutFolded;
                break;
            case initial:
                time.reset();
            case toHomeOutFolded:
                p = ArmPosition.HOME_OUT_FOLDED;
                followingState = State.toHomeOut;
                break;
            case toHomeOut:
                p = ArmPosition.HOME_OUT;
                followingState = State.homeOut;
                break;
        }
        if (p != null) {
            setArmPosition(p, followingState, currenttime);
        }
    }
}