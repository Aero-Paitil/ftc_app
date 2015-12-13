package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
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

    ElapsedTime time; //time for state machines
    Telemetry telemetry;
    ElapsedTime elbowTime; //time between elbow servo position updates

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
        elbowTime = new ElapsedTime();
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
        setElbowServoPosition(newPosition);
    }

    private void setElbowServoPosition(double pos) {
        if (elbowTime.time() > 0.3){
            elbowServo.setPosition(Range.clip(pos, 0, 1));
            elbowTime.reset();
        }

    }

    public void moveWrist(double positionChange) {
        double newPosition = wristServo.getPosition() + positionChange;
        wristServo.setPosition(Range.clip(newPosition, 0, 1));
    }

    public void moveTwist(double positionChange) {
        double newPosition = twistServo.getPosition() + positionChange;
        twistServo.setPosition(Range.clip(newPosition, 0, 1));
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
        setElbowServoPosition(p.elbow);
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

    // The methods below use forward and inverse kinematics
    // described in "Kinematics for Lynxmotion Robot Arm" by Dr. Rainer Hessman
    // http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf

    //Sangmin's edits on 12/6/2015
    public WristJointXY getCurrentXYPosition() {  //2pi= 3420 counts for Shoulder motor, Elbow motor pi = 180°
        double shoulderAngle = shoulderAngleFromCounts(shoulderMotor.getCurrentPosition());
        double elbowAngle = elbowAngleFromPosition(elbowServo.getPosition());
        DbgLog.msg("ARM current shoulder,elbow: " + shoulderMotor.getCurrentPosition() + ", " + elbowServo.getPosition());
        return new WristJointXY(shoulderAngle, elbowAngle);
    }
    /*
    *every x,y point of the wrist can be achieved by two ways: either elbow up or elbow down.
    * preferPlus is preffering positive elbow angle (or elbow down solution).
    *
    * We dont want to allow big elbow movements, so we want to check if it is safe to change between
    * elbow up and elbow down. It is safe to change when the arm is almost straight.
     */
    public boolean isSafeToChangePreferPlus()
    {
       double pos = elbowServo.getPosition();
        return (pos>=0.61 && pos<= 0.66);
    }

    public void changeXYPosition(double deltax, double deltay, boolean preferPlus) {
        WristJointXY currentXY = this.getCurrentXYPosition();
        double x = currentXY.getX() + deltax;
        double y = currentXY.getY() + deltay;
        DbgLog.msg("ARM deltaX,deltaY: " + deltax + ", " + deltay);
        DbgLog.msg("ARM new X,Y: " + x + ", " + y);


        double cosElbowAngle = (x * x + y * y - UPPERARM_LENGTH * UPPERARM_LENGTH - FOREARM_LENGTH * FOREARM_LENGTH) / (2 * UPPERARM_LENGTH * FOREARM_LENGTH);

        // plus solution
        double desiredElbowMotorPositionRadians_Plus = Math.atan2(Math.sqrt(1 - cosElbowAngle * cosElbowAngle), cosElbowAngle);
        double k1 = UPPERARM_LENGTH + FOREARM_LENGTH * Math.cos(desiredElbowMotorPositionRadians_Plus);
        double k2 = FOREARM_LENGTH * Math.sin(desiredElbowMotorPositionRadians_Plus);
        double desiredShoulderMotorPositionRadians_Plus = Math.atan2(y, x) - Math.atan2(k2, k1);
        int shoulderCounts_Plus = shoulderCountsFromAngle(desiredShoulderMotorPositionRadians_Plus);
        double elbowPos_Plus = elbowPositionFromAngle(desiredElbowMotorPositionRadians_Plus);
        DbgLog.msg("ARM Plus(Rad) shoulder, elbow: " + desiredShoulderMotorPositionRadians_Plus + ", " + desiredElbowMotorPositionRadians_Plus);
        DbgLog.msg("ARM Plus: " + shoulderCounts_Plus + ", " + elbowPos_Plus);

        // minus solution
        double desiredElbowMotorPositionRadians_Minus = Math.atan2(-Math.sqrt(1 - cosElbowAngle * cosElbowAngle), cosElbowAngle);
        k1 = UPPERARM_LENGTH + FOREARM_LENGTH * Math.cos(desiredElbowMotorPositionRadians_Minus);
        k2 = FOREARM_LENGTH * Math.sin(desiredElbowMotorPositionRadians_Minus);
        double desiredShoulderMotorPositionRadians_Minus = Math.atan2(y, x) - Math.atan2(k2, k1);
        int shoulderCounts_Minus = shoulderCountsFromAngle(desiredShoulderMotorPositionRadians_Minus);
        double elbowPos_Minus = elbowPositionFromAngle(desiredElbowMotorPositionRadians_Minus);
        DbgLog.msg("ARM Minus(Rad): " + desiredShoulderMotorPositionRadians_Minus + ", " + desiredElbowMotorPositionRadians_Minus);
        DbgLog.msg("ARM Minus: " + shoulderCounts_Minus + ", " + elbowPos_Minus);

        boolean usePlusSolution;
        boolean useMinusSolution;
        if (preferPlus) {
            usePlusSolution = shoulderCountsInRange(shoulderCounts_Plus) && elbowPositionInRange(elbowPos_Plus);
            useMinusSolution = false;

        } else {
            usePlusSolution = false;
            useMinusSolution = shoulderCountsInRange(shoulderCounts_Minus) && elbowPositionInRange(elbowPos_Minus);
        }

        if (usePlusSolution) {
            setShoulderPosition(shoulderCounts_Plus);
            setElbowServoPosition(elbowPos_Plus);
            DbgLog.msg("ARM setting shoulder,elbow " + shoulderCounts_Plus + ", " + elbowPos_Plus);
        } else if (useMinusSolution) {
            setShoulderPosition(shoulderCounts_Minus);
            setElbowServoPosition(elbowPos_Minus);
            DbgLog.msg("ARM setting shoulder,elbow " + shoulderCounts_Minus + ", " + elbowPos_Minus);
        } else {
            DbgLog.msg("ARM setting shoulder,elbow - OUT OF RANGE");
            telemetry.addData("ERROR", "Can not change position - out of the limits");
        }
    }

    private boolean shoulderCountsInRange(int shoulderCounts) {
        return (!Double.isNaN(shoulderCounts) && shoulderCounts < 130 && shoulderCounts > -1000);
    }

    private boolean elbowPositionInRange(double elbowPosition) {
        return (!Double.isNaN(elbowPosition) && elbowPosition > 0.39 && elbowPosition < 0.75);
    }

    /**
     * Shoulder Angle limits: from 130 to -1050 counts
     * calculating shoulder angle from encoder counts:
     * - counts * 2pi / (1140*3)
     *
     * @param counts encoder counts
     * @return shoulder angle
     */
    private double shoulderAngleFromCounts(int counts) {
        return -counts * 2 * Math.PI / (1140 * 3.0);
    }

    /**
     * calculating counts from shoulder angle:
     * - angle * 1140 * 3 / 2pi
     *
     * @param angle angle relative to horizontal
     * @return shoulder position in counts
     */
    private int shoulderCountsFromAngle(double angle) {
        return (int) (-angle * 1140 * 3 / (2 * Math.PI));
    }

    /**
     * (0 angle - 0.6353 servo position)
     * <p/>
     * calculating elbow angle from servo position
     * (servoPosition - 0.6353) * (K*pi)
     * <p/>
     * HS-785HB is a standard servo with a range of 3.5 rotations 1260 degrees
     * 0.1 change in servo position can correspond to 66-96 degrees change depending on shoulder and servo positions
     * we'll assume 0.1 change corresponds to 81 degrees
     * hence K = 81.0 * 10 / 180.0
     * <p/>
     * Elbow Angle limits: from 0.39 to 0.75 servo position
     * elbow angle zero is 0.64 servo position
     *
     * @param position position from servo
     * @return elbow angle from servo position
     */
    private double elbowAngleFromPosition(double position) {
        return (position - 0.6353) * Math.PI * (81.0 * 10 / 180.0);
    }

    /**
     * calculating servo position from elbow angle
     * angle/(K*pi) + 0.6353
     *
     * @param angle relative to horizontal
     * @return servo position from angle
     */
    private double elbowPositionFromAngle(double angle) {
        return angle * 180.0 / (Math.PI * (81.0 * 10)) + 0.6353;
    }

    public static class WristJointXY {
        private double x;
        private double y;

        public WristJointXY(double shoulderAngle, double elbowAngle) {
            this.x = UPPERARM_LENGTH * Math.cos(shoulderAngle) + FOREARM_LENGTH * Math.cos(shoulderAngle + elbowAngle);
            this.y = UPPERARM_LENGTH * Math.sin(shoulderAngle) + FOREARM_LENGTH * Math.sin(shoulderAngle + elbowAngle);
            DbgLog.msg("ARM current x,y: " + this.x + ", " + this.y);
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }
}