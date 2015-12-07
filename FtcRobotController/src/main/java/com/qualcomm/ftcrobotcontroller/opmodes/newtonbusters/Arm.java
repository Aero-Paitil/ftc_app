package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by jasmine on 11/28/15
 * converting from hardware values to software values-
 * SERVOS: hardware values: 0-255
 * SERVOS: software values: 0-1
 * OPTION COMMAND L TO REFORMAT
 */
public class Arm {

    static final double TASK_TIME = 1.0;
    static final double HOLD_POSITION_POWER = 0.3;
    static final double NO_TWIST_POSITION = 0.45;

    static final double UPPERARM_LENGTH = 12.5; // inches
    static final double FOREARM_LENGTH = 12.5; // inches


    enum State {initial, toHomeIn, homeIn, toHomeInFolded, toHomeOutFolded, toHomeOut, homeOut, homeOutToFront, toFront, atFront, homeOutToPeopleDrop, toPeopleDrop, atPeopleDrop}

    State armState;
    DcMotor shoulderMotor;
    Servo elbowServo, wristServo, twistServo;

    ElapsedTime time;
    Telemetry telemetry;

    public static class XY {
        private double x;
        private double y;
        public XY(double x, double y){
            x = this.x;
            y = this.y;
        }
        public double getX(){
            return x;
        }
        public double getY(){
            return y;
        }
    }

    public static class ArmPosition {
        //static ArmPosition DRIVER_LOW_LIMIT = new ArmPosition(-100, 0/255, 0);
        //static ArmPosition DRIVER_HIGH_LIMIT = new ArmPosition(-850, 195.0/255, 1);
        //TODO: CHECK ALL POSITIONS!!!
        static ArmPosition INITIAL = new ArmPosition(0, 0.41, 1);
        //static ArmPosition HOME_IN_FINAL = new ArmPosition(-122, 0.455, 0.81); //more back
        static ArmPosition HOME_IN_FINAL = new ArmPosition(-172, 0.46, 0.75);
        static ArmPosition HOME_IN = new ArmPosition(-172, 0.46, 1); // wrist is adjusted in the end
        static ArmPosition HOME_IN_FOLDED = new ArmPosition(-158, 0.41, 1);
        static ArmPosition HOME_OUT_FOLDED = new ArmPosition(-600, 0.41, 1);
        static ArmPosition HOME_OUT = new ArmPosition(-600, 0.49, 1);
        static ArmPosition HOME_OUT_TO_FRONT = new ArmPosition(-400, 0.6, 0.501);
        static ArmPosition IN_FRONT = new ArmPosition(-235, 0.517, 0.501);
        static ArmPosition HOME_OUT_TO_PEOPLE_DROP = new ArmPosition(-520, 0.61, 0.7);
        static ArmPosition PEOPLE_DROP = new ArmPosition(-510, 0.75, 0); //0.665 wrist to drop

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
        twistServo.setPosition(NO_TWIST_POSITION);
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
     *
     * @param speed is from -1 to 1
     */
    public void moveShoulder(double speed) {
        if (Math.abs(speed) < 0.1) {
            holdShoulderPosition();
        }
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        shoulderMotor.setPower(speed / 10);
    }

    public void changeShoulderPosition(int byCounts) {
        int currentPosition = shoulderMotor.getCurrentPosition();
        int newPosition = currentPosition + byCounts;
        setShoulderPosition(newPosition);
    }

    public void moveElbow(double positionChange) {
        double newPosition = elbowServo.getPosition() + positionChange;
        elbowServo.setPosition(Range.clip(newPosition, 0, 1));
    }

    public void moveWrist(double positionChange) {
        double newPosition = wristServo.getPosition() + positionChange;
        wristServo.setPosition(Range.clip(newPosition, 0, 1));
    }

    public void moveTwist(double positionChange) {
        double newPosition = twistServo.getPosition() + positionChange;
        twistServo.setPosition(Range.clip(newPosition, 0, 1));
    }

    //Sangmin's edits on 12/6/2015
    public XY getCurrentXYPostion(){  //2pi= 3420 counts for Shoulder motor, Elbow motor pi = 180°
        double shoulderAngle = shoulderMotor.getCurrentPosition() * 2*Math.PI / 3420;
        double elbowAngle = elbowServo.getPosition() * Math.PI;
        return new XY((UPPERARM_LENGTH*Math.cos(shoulderAngle)) + (FOREARM_LENGTH* Math.cos(shoulderAngle + elbowAngle)), UPPERARM_LENGTH*Math.sin(shoulderAngle) + FOREARM_LENGTH*Math.sin(shoulderAngle+elbowAngle));
    }
    public void changeXYPosition(double deltax, double deltay){
        double x = this.getCurrentXYPostion().getX() + deltax;
        double y = this.getCurrentXYPostion().getY() + deltay;
        double desiredShoulderMotorPositionRadians_Plus = Math.atan2(Math.sqrt(1 - Math.pow((Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(UPPERARM_LENGTH, 2) - Math.pow(FOREARM_LENGTH, 2)) / (2 * FOREARM_LENGTH * UPPERARM_LENGTH), 2))
                , (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(UPPERARM_LENGTH, 2) - Math.pow(FOREARM_LENGTH, 2)) / (2 * UPPERARM_LENGTH * FOREARM_LENGTH));
        double desiredElbowMotorPositionRadians_Plus = Math.atan2(y, x) - Math.atan2(UPPERARM_LENGTH + FOREARM_LENGTH * Math.cos(desiredShoulderMotorPositionRadians_Plus), FOREARM_LENGTH*Math.sin(desiredShoulderMotorPositionRadians_Plus));
        /*
        double desiredShoulderMotorPositionRadians_Minus = Math.atan2(-Math.sqrt(1-Math.pow((Math.pow(x,2) + Math.pow(y,2) - Math.pow(UPPERARM_LENGTH,2) - Math.pow(FOREARM_LENGTH, 2))/(2*FOREARM_LENGTH*UPPERARM_LENGTH), 2))
        , (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(UPPERARM_LENGTH, 2) - Math.pow(FOREARM_LENGTH, 2))/(2*UPPERARM_LENGTH*FOREARM_LENGTH));
        double desiredElbowMotorPositionRadians_Minus = Math.atan2(y, x) - Math.atan2(UPPERARM_LENGTH + FOREARM_LENGTH * Math.cos(desiredShoulderMotorPositionRadians_Plus), FOREARM_LENGTH*Math.sin(desiredShoulderMotorPositionRadians_Plus));
         */
        shoulderMotor.setTargetPosition((int)(desiredShoulderMotorPositionRadians_Plus * 3420 / (2*Math.PI)));
        elbowServo.setPosition((int)desiredElbowMotorPositionRadians_Plus);
    }


    public void toPeopleDropPosition() {

        if (armState == State.atPeopleDrop) {
            return;
        }
        ArmPosition p = null;
        State followingState = null;
        double currenttime = time.time();
        switch (armState) {
            case homeOut:
                twistServo.setPosition(NO_TWIST_POSITION);
                armState = State.homeOutToPeopleDrop;
                time.reset();
                break;
            case homeOutToPeopleDrop:
                p = ArmPosition.HOME_OUT_TO_PEOPLE_DROP;
                followingState = State.toPeopleDrop;
                break;
            case toPeopleDrop:
                p = ArmPosition.PEOPLE_DROP;
                followingState = State.atPeopleDrop;
                break;
            default:
                // we should start at home out
                break;
        }
        if (p != null) {
            setArmPosition(p, followingState, currenttime);
        }
    }


    public void toFrontPosition() {

        if (armState == State.atFront) {
            return;
        }
        ArmPosition p = null;
        State followingState = null;
        double currenttime = time.time();
        switch (armState) {
            case homeOut:
                twistServo.setPosition(NO_TWIST_POSITION);
                armState = State.homeOutToFront;
                time.reset();
                break;
            case homeOutToFront:
                p = ArmPosition.HOME_OUT_TO_FRONT;
                followingState = State.toFront;
                break;
            case toFront:
                p = ArmPosition.IN_FRONT;
                followingState = State.atFront;
                break;
            default:
                // we should start at home out
                break;
        }
        if (p != null) {
            setArmPosition(p, followingState, currenttime);
        }
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
                twistServo.setPosition(NO_TWIST_POSITION);
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
            default:
                twistServo.setPosition(NO_TWIST_POSITION);
                p = ArmPosition.HOME_OUT;
                followingState = State.homeOut;
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
                twistServo.setPosition(NO_TWIST_POSITION);
                armState = State.toHomeInFolded;
                time.reset();
                break;
            case toHomeInFolded:
                p = ArmPosition.HOME_IN_FOLDED;
                followingState = State.toHomeOutFolded;
                break;
            case initial:
                armState = State.toHomeOutFolded;
                time.reset();
                break;
            case toHomeOutFolded:
                p = ArmPosition.HOME_OUT_FOLDED;
                followingState = State.toHomeOut;
                break;
            case toHomeOut:
            default:
                twistServo.setPosition(NO_TWIST_POSITION);
                p = ArmPosition.HOME_OUT;
                followingState = State.homeOut;
                break;
        }
        if (p != null) {
            setArmPosition(p, followingState, currenttime);
        }
    }

    State getArmState() {
        return armState;
    }

    public void autonomousUndockArm() {
        setArmPosition(ArmPosition.HOME_IN_FOLDED, null, 0);
        Utils.delay(TASK_TIME);
        setArmPosition(ArmPosition.HOME_OUT_FOLDED, null, 0);
    }
}