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
    DcMotor shoulderMotor;

    Servo rightButtonPusher, leftButtonPusher;
    Servo skiLiftHandleRight, skiLiftHandleLeft;
    Servo frontSweeper;
    Servo spool1;
    Servo spool2;
    Servo peopleDrop;

    enum SweeperState {Undeployed, BarForward, StartingBrushes, Deployed, StoppingBrushes, BarBack}

    SweeperState sweeperState;
    ElapsedTime sweeperTimer;


    //Servo peopleDrop;

    boolean rightClimberReleased, leftClimberReleased;

    boolean movingShoulderBig, movingShoulderSmall;

    boolean movingRearWheelsUp, movingRearWheelsHoldingPos;

    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.

        rearWheels = hardwareMap.dcMotor.get("RearWheels");
        rearWheels.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        brush = hardwareMap.dcMotor.get("Brush");

        shoulderMotor = hardwareMap.dcMotor.get("Arm");
        shoulderMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        movingShoulderSmall = false;
        movingShoulderBig = false;

    }

    @Override
    public void start() {

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
        frontSweeper.setPosition(121.0 / 255);
        sweeperState = SweeperState.Undeployed;
        sweeperTimer = new ElapsedTime();

        //Zero is the dropping position, 1 is the initial position.
        peopleDrop = hardwareMap.servo.get("PeopleDrop");
        peopleDrop.setPosition(1);


        spool1 = hardwareMap.servo.get("Spool1");
        spool1.setPosition(0.5);
        spool2 = hardwareMap.servo.get("Spool2");
        spool2.setPosition(0.5);

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
        if (gamepad1.a || gamepad1.b) {
            if (!movingRearWheelsHoldingPos) {
                rearWheels.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rearWheels.setPower(1);
                movingRearWheelsHoldingPos = true;
            }

            int finalPosition;
            if (gamepad1.a) {
                finalPosition = rearWheelsPosition + 100;
            } else { //gamepad1.b
                finalPosition = rearWheelsPosition - 100;
            }
            rearWheels.setTargetPosition(finalPosition);

        } else {
            if (movingRearWheelsHoldingPos) {
                movingRearWheelsHoldingPos = false;
            }
        }
        if (gamepad1.y) {
            if (!movingRearWheelsUp) {
                rearWheels.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                rearWheels.setPower(-0.3);
                movingRearWheelsUp = true;
            }
        } else {
            if (movingRearWheelsUp) {
                rearWheels.setPower(0);
                movingRearWheelsUp = false;
            }
        }

        contolArm();

        // frontSweeperDeployed, brushDeployed;
        if (gamepad1.start) {
            deploySweeper();
        } else if (gamepad1.guide) {
            undeploySweeper();
        }

        telemetry.addData("spool1", spool1.getConnectionInfo());
        telemetry.addData("spool2", spool2.getConnectionInfo());

        // controls spools
        if (gamepad2.y) {
            spool1.setPosition(0);
            spool2.setPosition(0);
        } else if (gamepad2.a) {
            //extend
            spool1.setPosition(1);
            spool2.setPosition(1);
        } else {
            spool1.setPosition(0.5);
            spool2.setPosition(0.5);

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
                    frontSweeper.setPosition(118 / 255d);
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

    void contolArm() {
        telemetry.addData("Shoulder", shoulderMotor.getCurrentPosition());

        // small shoulder adjustments
        if (!movingShoulderBig) {
            if (gamepad2.left_trigger > 0.5) {
                if (!movingShoulderSmall) {
                    changeShoulderPosition(10);
                    movingShoulderSmall = true;
                }
            } else if (gamepad2.right_trigger > 0.5) {
                if (!movingShoulderSmall) {
                    changeShoulderPosition(-10);
                    movingShoulderSmall = true;
                }
            } else if (movingShoulderSmall) {
                movingShoulderSmall = false;
            }
        }


        // big shoulder adjustments
        if (!movingShoulderSmall) {
            if (gamepad2.right_stick_y != 0 && Math.abs(gamepad2.right_stick_y) > 0.1) { //negative is up
                moveShoulder(gamepad2.right_stick_y);
                movingShoulderBig = true;
            } else if (movingShoulderBig) {
                holdShoulderPosition();
                movingShoulderBig = false;
            }
        }

    }

    public void changeShoulderPosition(int byCounts) {
        int currentPosition = shoulderMotor.getCurrentPosition();
        int newPosition = currentPosition + byCounts;
        setShoulderPosition(newPosition);
    }

    private void setShoulderPosition(int position) {
        shoulderMotor.setTargetPosition(position);
        shoulderMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        shoulderMotor.setPower(0.3);
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
}
