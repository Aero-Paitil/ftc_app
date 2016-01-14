package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

//import com.qualcomm.ftccommon.DbgLog;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Aryoman on 11/24/2015.
 * This class has the code for our driver controlled mode.
 */
public class DriverOpMode extends OpMode {
    final static private double CLIMBER_RELEASE_POS_RIGHT = 195 / 255d;
    final static private double CLIMBER_RELEASE_POS_LEFT = 90 / 255d;

    MecanumWheels mecanumWheels;
    DcMotor rearWheels;
    DcMotor brush;

    Servo rightButtonPusher, leftButtonPusher;
    Servo skiLiftHandleRight, skiLiftHandleLeft;
    Servo frontSweeper;

    enum SweeperState {Undeployed, BarForward, StartingBrushes, Deployed, StoppingBrushes, BarBack}

    SweeperState sweeperState;
    ElapsedTime sweeperTimer;


    //Servo peopleDrop;

    boolean rightClimberReleased, leftClimberReleased;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.

        rearWheels = hardwareMap.dcMotor.get("RearWheels");
        rearWheels.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        brush = hardwareMap.dcMotor.get("Brush");
    }

    @Override
    public void start() {

        //rearWheels.setTargetPosition(-REAR_WHEELS_COUNTS);
        rearWheels.setTargetPosition(0);
        rearWheels.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rearWheels.setPower(0.3);

        rightButtonPusher = hardwareMap.servo.get("RightButtonPusher");
        leftButtonPusher = hardwareMap.servo.get("LeftButtonPusher");
        //0 servo setting means touch pusher isn't deployed, all the way in, didn't touch/sense anything
        //1 servo setting means touch pusher is deployed, and touched/sensed something
        leftButtonPusher.setDirection(Servo.Direction.REVERSE);
        rightButtonPusher.setPosition(0.5);
        leftButtonPusher.setPosition(0.5);

        skiLiftHandleRight = hardwareMap.servo.get("SkiLiftHandleRight");
        //value 45/255 is the initial position, value 195/255 is the deployed position
        skiLiftHandleRight.setPosition(45.0 / 255);
        skiLiftHandleLeft = hardwareMap.servo.get("SkiLiftHandleLeft");
        //value 240/255 is the initial position, value 90/255 is the deployed position
        skiLiftHandleLeft.setPosition(240.0 / 255);
        frontSweeper = hardwareMap.servo.get("FrontSweeper");
        //value 200/255 is the initial position, value 100/255 is the deployed position
        frontSweeper.setPosition(125.0 / 255);
        sweeperState = SweeperState.Undeployed;
        sweeperTimer = new ElapsedTime();

    }

    @Override
    public void loop() {

        //changing field coordinates
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            mecanumWheels.resetGyroHeading();
            while (true) {
                if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    break;
                }
            }
        } else {
            //releasing climbers
            if (gamepad1.right_bumper) {
                if (!rightClimberReleased) {
                    skiLiftHandleRight.setPosition(CLIMBER_RELEASE_POS_RIGHT);
                    rightClimberReleased = true;
                } else {
                    skiLiftHandleRight.setPosition(45 / 255d);
                    rightClimberReleased = false;
                }
                while (true) {
                    if (!gamepad1.right_bumper) {
                        break;
                    }
                }
            } else if (gamepad1.left_bumper) {
                if (!leftClimberReleased) {
                    skiLiftHandleLeft.setPosition(CLIMBER_RELEASE_POS_LEFT);
                    leftClimberReleased = true;
                } else {
                    skiLiftHandleLeft.setPosition(240 / 255d);
                    leftClimberReleased = false;
                }
                while (true) {
                    if (!gamepad1.left_bumper) {
                        break;
                    }
                }

            }

        }

        double clockwise = gamepad1.right_stick_x; //doesn't matter field or robot

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            //We are using robot coordinates
            double forward = 0;
            double right = 0;
            if (gamepad1.dpad_up) {
                forward = 1;
            } else if (gamepad1.dpad_down) {
                forward = -1;
            }
            if (gamepad1.dpad_right) {
                right = 1;
            } else if (gamepad1.dpad_left) {
                right = -1;
            }
            mecanumWheels.powerMotors(forward, right, clockwise, false);//Not using gyro
        } else {
            //We are using the joystick values in the driver's perspective (field coordinates).
            double fieldForward = -gamepad1.left_stick_y; //field coordinates
            double fieldRight = gamepad1.left_stick_x; //field coordinates
            mecanumWheels.powerMotors(fieldForward, fieldRight, clockwise, true);
        }

        //this code controls the rear wheels
        //todo check limits
        int rearWheelsPosition = rearWheels.getCurrentPosition();
        telemetry.addData("rear wheel position", rearWheelsPosition);
        if (gamepad1.a) {

            int finalPosition = rearWheelsPosition + 100;
            if (finalPosition < 4000) {
                rearWheels.setTargetPosition(finalPosition);
            }
        }
        if (gamepad1.b) {
            int finalPosition = rearWheelsPosition - 100;
            if (finalPosition >= 0) {
                rearWheels.setTargetPosition(finalPosition);
            }
        }


        // frontSweeperDeployed, brushDeployed;
        if (gamepad1.start) {
            deploySweeper();
        } else if (gamepad1.guide) {
            undeploySweeper();
        }
    }

    void deploySweeper() {
        switch (sweeperState) {
            case Undeployed:
            case BarBack:
                frontSweeper.setPosition(105 / 255d);
                sweeperTimer.reset();
                sweeperState = SweeperState.BarForward;
                break;
            case BarForward:
            case StoppingBrushes:
                if (sweeperTimer.time() > 1.5) {
                    brush.setPower(1);
                    sweeperTimer.reset();
                    sweeperState = SweeperState.StartingBrushes;
                }
                break;
            case StartingBrushes:
                if (sweeperTimer.time() > 0.5) {
                    sweeperState = SweeperState.Deployed;
                }
                break;
            case Deployed:
                break;
        }
    }

    //Undeployed, BarForward, StartingBrushes, Deployed, StoppingBrushes, BarBack
    void undeploySweeper() {
        switch (sweeperState) {
            case Deployed:
            case StartingBrushes:
                brush.setPower(0);
                sweeperTimer.reset();
                sweeperState = SweeperState.StoppingBrushes;
                break;
            case StoppingBrushes:
            case BarForward:
                if (sweeperTimer.time() > 0.5) {
                    frontSweeper.setPosition(125 / 255d);
                    sweeperTimer.reset();
                    sweeperState = SweeperState.BarBack;
                }
                break;
            case BarBack:
                if (sweeperTimer.time() > 1.5) {
                    sweeperState = SweeperState.Undeployed;
                }
                break;

        }
    }
}