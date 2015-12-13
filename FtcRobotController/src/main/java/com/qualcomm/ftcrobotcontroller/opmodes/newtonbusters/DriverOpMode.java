package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;
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
    final static private int REAR_WHEELS_COUNTS = 1000;
    final static private double CLIMBER_RELEASE_POS = 1.0;

    MecanumWheels mecanumWheels;
    Brushes brushes;
    DcMotor rearWheels;
    //prefer elbow down
    private boolean preferElbowDown;

    enum BrushState {inward, outward, stopped}

    BrushState brushState;
    static final double POS_UPDATE_TIME = 0.1;
    Servo extension4, extension5, beacon6, skiLiftHandleRight, skiLiftHandleLeft;
    Arm arm;
    ElapsedTime posUpdateTime;
    boolean rightClimberReleased, leftClimberReleased;

    boolean movingShoulderSmall = false, movingShoulderBig = false;

    boolean movingWrist = false;
    ElapsedTime wristTime;


    @Override
    public void init() {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true); //We are using the Gyro.

        rearWheels = hardwareMap.dcMotor.get("RearWheels");
        rearWheels.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        posUpdateTime = new ElapsedTime();
        arm = new Arm(hardwareMap, telemetry);
        movingShoulderSmall = false;
        movingShoulderBig = false;

        extension4 = hardwareMap.servo.get("Extension4");
        extension4.setPosition(1.0);
        extension5 = hardwareMap.servo.get("Extension5");
        extension5.setPosition(0.0);
        beacon6 = hardwareMap.servo.get("Beacon6");
        beacon6.setPosition(0.0);

        movingWrist = false;
        wristTime = new ElapsedTime();

        preferElbowDown = false;
    }

    @Override
    public void start() {
        brushes = new Brushes(hardwareMap, telemetry);
        brushState = BrushState.stopped;
        skiLiftHandleRight = hardwareMap.servo.get("SkiLiftHandle6");
        skiLiftHandleRight.setPosition(0.1);
        rightClimberReleased = false;
        skiLiftHandleLeft = hardwareMap.servo.get("SkiLiftHandle5");
        skiLiftHandleLeft.setDirection(Servo.Direction.REVERSE);
        skiLiftHandleLeft.setPosition(0.1);
        leftClimberReleased = false;

        //rearWheels.setTargetPosition(-REAR_WHEELS_COUNTS);
        rearWheels.setTargetPosition(0);
        rearWheels.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rearWheels.setPower(0.3);
        arm.holdShoulderPosition();
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
                    skiLiftHandleRight.setPosition(CLIMBER_RELEASE_POS);
                    rightClimberReleased = true;
                } else {
                    skiLiftHandleRight.setPosition(0.0);
                    rightClimberReleased = false;
                }
                while (true) {
                    if (!gamepad1.right_bumper) {
                        break;
                    }
                }
            } else if (gamepad1.left_bumper) {
                if (!leftClimberReleased) {
                    skiLiftHandleLeft.setPosition(CLIMBER_RELEASE_POS);
                    leftClimberReleased = true;
                } else {
                    skiLiftHandleLeft.setPosition(0.0);
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

        //this code controls the brushes
        brushes.printTelemetry();

        if (gamepad1.start) {
            brushes.undockBrushes();
        } else if (gamepad1.guide) {
            brushes.dockBrushes();
        } else if (gamepad2.guide) {
            //stop the movement of the brushes
            brushes.setRotation(Brushes.STOP_ROTATION);
            brushState = BrushState.stopped;
        } else if (gamepad2.start) {
            if (brushState != BrushState.inward) {
                //rotate brushes inward
                brushes.setRotation(Brushes.ROTATE_INWARD_FAST);
                brushState = BrushState.inward;
            } else {
                //rotate brushes outward
                brushes.setRotation(Brushes.ROTATE_OUTWARD_FAST);
                brushState = BrushState.outward;
            }
            while (true) {
                if (!gamepad2.start) break;
            }
        }

        //this code controls the rear wheels
        //todo check limits
        telemetry.addData("rear wheel position", rearWheels.getCurrentPosition());
        if (gamepad1.a) {
            rearWheels.setTargetPosition(rearWheels.getCurrentPosition() + 500);
        }
        if (gamepad1.b) {
            rearWheels.setTargetPosition(rearWheels.getCurrentPosition() - 500);
        }

        contolArm();

    }

    void contolArm() {
        arm.telemetry();

        //wrist & twist
        if (movingWrist && wristTime.time()>1){
            if (gamepad2.y) {
                arm.moveWrist(0.01);
            } else if (gamepad2.a) {
                arm.moveWrist(-0.01);
            } else if (gamepad2.x) {
                arm.moveTwist(0.01);
            } else if (gamepad2.b) {
                arm.moveTwist(-0.01);
            } else {
                movingWrist = false;
            }
        } else {
            if (gamepad2.y) {
                if (!movingWrist) {
                    arm.moveWrist(0.01);
                    movingWrist = true;
                    wristTime.reset();
                }
            } else if (gamepad2.a) {
                if (!movingWrist) {
                    arm.moveWrist(-0.01);
                    movingWrist = true;
                    wristTime.reset();
                }
            } else if (gamepad2.x) {
                if (!movingWrist) {
                    arm.moveTwist(0.01);
                    movingWrist = true;
                    wristTime.reset();
                }
            } else if (gamepad2.b) {
                if (!movingWrist) {
                    arm.moveTwist(-0.01);
                    movingWrist = true;
                    wristTime.reset();
                }
            } else {
                if (movingWrist) {
                    movingWrist = false;
                }
            }
        }
        telemetry.addData("preferElbowDown", preferElbowDown);
        if (gamepad2.left_stick_button)
        {
            if (arm.isSafeToChangePreferPlus())
            {
                preferElbowDown = !preferElbowDown;
                while (true)
                {
                    if (!gamepad2.left_stick_button)
                    {
                        break;
                    }
                }
            }
            else
            {
                DbgLog.msg("ERROR: Not safe to switch between elbow up and down");
            }
        }

        //We have two ways to control the arm:
        //
        //1)   The left joystick defines x,y coordinates of the wrist,
        //so that shoulder and elbow angles are moving together,
        //
        //2)  The right joystick controls shoulder and elbow angles separately,
        //so that x controlls the elbow angle and y controlls the shoulder angle.
        if (Math.abs(gamepad2.left_stick_x) >= 0.5 || Math.abs(gamepad2.left_stick_y) >= 0.5)
        {
            if (posUpdateTime.time()>0.3)
            {
                DbgLog.msg("ARM joystick X,Y" + gamepad2.left_stick_x + ", " + -gamepad2.left_stick_y);
                arm.changeXYPosition(gamepad2.left_stick_x, -gamepad2.left_stick_y, preferElbowDown);
                posUpdateTime.reset();
            }
        }
        else {
            //elbow
            if (gamepad2.right_stick_x != 0) { //negative is up
                if (posUpdateTime.time() > POS_UPDATE_TIME) {
                    arm.moveElbow(gamepad2.right_stick_x / 100);
                    posUpdateTime.reset();
                }
            }

            //shoulder

            // small adjustments
            if (!movingShoulderBig) {
                if (gamepad2.left_trigger > 0.5) {
                    if (!movingShoulderSmall) {
                        arm.changeShoulderPosition(10);
                        movingShoulderSmall = true;
                    }
                } else if (gamepad2.right_trigger > 0.5) {
                    if (!movingShoulderSmall) {
                        arm.changeShoulderPosition(-10);
                        movingShoulderSmall = true;
                    }
                } else if (movingShoulderSmall) {
                    movingShoulderSmall = false;
                }
            }


            // big adjustments
            if (!movingShoulderSmall) {
                if (gamepad2.right_stick_y != 0 && Math.abs(gamepad2.right_stick_y) > 0.1) { //negative is up
                    arm.moveShoulder(gamepad2.right_stick_y);
                    movingShoulderBig = true;
                } else if (movingShoulderBig) {
                    arm.holdShoulderPosition();
                    movingShoulderBig = false;
                }
            }
        }

        if (gamepad2.dpad_up) {
            // make sure the brushes are undocked when moving from initial position
            if (arm.getArmState() != Arm.State.initial || brushes.getState() == Brushes.State.brushesUndocked) {
                arm.undockArm();
            } else {
                brushes.undockBrushes();
            }
        } else if (gamepad2.dpad_down) {
            // make sure the brushes are undocked before docking the arm
            if (brushes.getState() == Brushes.State.brushesUndocked) {
                arm.dockArm();
            } else {
                brushes.undockBrushes();
            }
        } else if (gamepad2.dpad_right) {
            arm.toPeopleDropPosition();

        } else if (gamepad2.dpad_left) {
            // make sure the brushes are docked before using in-front-of-brushes position
            if (brushes.getState() == Brushes.State.brushesDocked) {
                arm.toFrontPosition();
            } else {
                brushes.dockBrushes();
            }
        }
    }

}