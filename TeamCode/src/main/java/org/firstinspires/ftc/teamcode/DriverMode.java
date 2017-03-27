package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverMode", group="nb")
//@Disabled

public class DriverMode extends OpMode {

    //public static final String ALLIANCE = "RED";
//    private ElapsedTime timer = new ElapsedTime();
//    private ElapsedTime motorleftTimer = new ElapsedTime();
//    private ElapsedTime motorrightTimer = new ElapsedTime();
//    private double lastLeftForward = 0;
//    private double lastRightForward = 0;

//    private static final String TAG = "Driver Mode";

    //defining the 4 motors
    private DcMotor motorLeft1;
    private DcMotor motorRight1;

    private DcMotor motorFlywheel;
    //private DcMotor motorFlywheelLeft;

    private DcMotor motorBelt;
    private DcMotor motorBrush;

    private double lightIntensity = 0.75;
    private DcMotor lightStrip1; //closer to blue wheels
    private DcMotor lightStrip2; //closer to intake mechanism
    private DcMotor lightStripGun;

    private Servo servoBar;
    private Servo kickServo2;
    private Servo kickServo3;

    private boolean inBrushBtnPressed = false;
    private boolean outBrushBtnPressed = false;
    private int brushState = 0; //-1:Intake, 1:Sweep out, 0:Stopped
    private ElapsedTime gunTimer = new ElapsedTime();
    private boolean triggered = false; //triggered= trigger pressed

    private boolean isRedAlliance = true;
    private double allianceColor = isRedAlliance ? lightIntensity : -lightIntensity;

    private boolean forward;
    private boolean backButtonPressed = false;

    private static double MINPOWER = 0.15;

    private double scaled(double x) {
        return (x / 1.07) * (.62 * x * x + .45);
    }

    private enum Flywheel {off, speedUp1, speedUp2, readyToShoot}

    private Flywheel flywheelState = Flywheel.off;

    @Override
    public void init() {
        triggered = false;
        flywheelState = Flywheel.off;
        //"initializing" the motors
        motorLeft1 = hardwareMap.dcMotor.get("D1left");
        motorRight1 = hardwareMap.dcMotor.get("D1right");
        // run by power
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // float zero power
        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lightStrip1 = hardwareMap.dcMotor.get("lightStrip1");
        lightStrip2 = hardwareMap.dcMotor.get("lightStrip2");
        lightStripGun = hardwareMap.dcMotor.get("lightStripGun");

        motorBrush = hardwareMap.dcMotor.get("Brush");
        motorBelt = hardwareMap.dcMotor.get("Belt");
        motorFlywheel = hardwareMap.dcMotor.get("GunRight");
        //motorFlywheelLeft = hardwareMap.dcMotor.get("GunLeft");
        // run flywheels by speed
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // motorFlywheelLeft.setZeroPowerBehavior(Dc Motor.ZeroPowerBehavior.FLOAT);

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);

        forward = true;
    }

    public void start() {
        Servo servoBeaconPad = hardwareMap.servo.get("BeaconPad");
        servoBeaconPad.setPosition(15 / 255.0);
        servoBar = hardwareMap.servo.get("ServoBar");
        servoBar.setPosition(225.0 / 255);
        kickServo2 = hardwareMap.servo.get("KickServo2");
        kickServo2.setPosition(10.0 / 255);
        kickServo3 = hardwareMap.servo.get("KickServo3");
        kickServo3.setPosition(215.0 / 255);
        lightStrip2.setPower(lightIntensity);
    }


    @Override
    public void loop() {

        //Changing Brush State
        if (gamepad1.x) {
            inBrushBtnPressed = true;
        } else if (gamepad1.b) {
            outBrushBtnPressed = true;

        } else {
            if (inBrushBtnPressed) {
                inBrushBtnPressed = false;
                brushState = (brushState == 0) ? -1 : 0;
            } else if (outBrushBtnPressed) {
                outBrushBtnPressed = false;
                brushState = (brushState == 0) ? 1 : 0;
            }
        }
        if (brushState == -1) {
            motorBrush.setPower(1);
            telemetry.addData("Brush", "Intake");
        } else if (brushState == 1) {
            motorBrush.setPower(-1);
            telemetry.addData("Brush", "Sweep Out");
            motorBelt.setPower(-1);
            telemetry.addData("Belt", "down");
        } else {
            motorBrush.setPower(0);
            telemetry.addData("Brush", "Stopped");
        }

        double leftForward;
        double rightForward;

        int sign = forward ? 1 : -1;
        if (!gamepad1.y) {
            //Arcade Drive
            rightForward = -(scaled(gamepad1.left_stick_y) + sign * scaled(gamepad1.left_stick_x));
            leftForward = -(scaled(gamepad1.left_stick_y) - sign * scaled(gamepad1.left_stick_x));
        } else {
            //Tank Drive
            leftForward = -scaled(gamepad1.left_stick_y);
            rightForward = -scaled(gamepad1.right_stick_y);
        }

        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)) {
            // dPadDrive: Forward, Backwards, Rotate in place CClockwise and Clockwise
            //We are using robot coordinates
            double dpadSpeed = 0.2;
            if (gamepad1.dpad_up) {
                rightForward = dpadSpeed;
                leftForward = dpadSpeed;
            } else if (gamepad1.dpad_down) {
                rightForward = -dpadSpeed;
                leftForward = -dpadSpeed;
            } else if (gamepad1.dpad_left) {
                rightForward = sign * dpadSpeed;
                leftForward = -sign * dpadSpeed;
            } else {
                leftForward = sign * dpadSpeed;
                rightForward = -sign * dpadSpeed;
            }
        }


        //using the Gun
        handleFlywheel();

        //using the Belt
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            if (flywheelState != Flywheel.readyToShoot) {
                motorBelt.setPower(0);
            } else {
                motorBelt.setPower(1);
                telemetry.addData("Belt", "Up");
            }
        } else {
            if (brushState != 1) {
                motorBelt.setPower(0);
                telemetry.addData("Belt", "Off");
            }
        }

        //driving backwards
        if (!forward) { //when start, front direction is the intake side, lightStrip2
            leftForward = -leftForward;
            rightForward = -rightForward;

        }

        //todo adjust the deadband
        if ((rightForward > -0.01) && (rightForward < 0.01))
            rightForward = 0;
        else if ((rightForward > -MINPOWER) && (rightForward < MINPOWER))
            rightForward = MINPOWER * rightForward / Math.abs(rightForward);

        if ((leftForward > -0.01) && (leftForward < 0.01))
            leftForward = 0;
        else if ((leftForward > -MINPOWER) && (leftForward < MINPOWER))
            leftForward = MINPOWER * leftForward / Math.abs(leftForward);

        rightForward = Range.clip(rightForward, -1, 1);
        leftForward = Range.clip(leftForward, -1, 1);

        //Changing direction from forward to backward and backward to forward
        //Gamepad back button does not work with motorola 3G
        if (gamepad1.a) {
            backButtonPressed = true;
        } else if (backButtonPressed) {
            backButtonPressed = false;
            forward = !forward;
            lightSwitch();
        }

        if (gamepad1.back) {
            if (allianceColor == lightIntensity) {
                isRedAlliance = false;
            } else { //allianceColor == -lightIntensity
                isRedAlliance = true;
            }
        }

        telemetry.addData("left", leftForward);
        telemetry.addData("right", rightForward);

        telemetry.addData("back(Y) button pressed", backButtonPressed);
        telemetry.addData("forward", forward);


        /* assigning the motors the scaled powers that we just calculated in the step above. */
        powerMotors(rightForward, leftForward);

    }

    private void lightSwitch() {
        if (forward) {
            lightStrip1.setPower(0);
            lightStrip2.setPower(lightIntensity);
        } else {
            lightStrip1.setPower(lightIntensity);
            lightStrip2.setPower(0);
        }
    }

    private void powerMotors(double rightForward, double leftForward) {
        motorLeft1.setPower(leftForward);
        motorRight1.setPower(rightForward);
    }


    private void handleFlywheel() {
        if (gamepad1.left_trigger > 0.7 || gamepad1.right_trigger > 0.7) {
            if (!triggered) {
                if (flywheelState == Flywheel.off) {
                    gunTimer.reset();
                    motorFlywheel.setPower(-0.5);
                    flywheelState = Flywheel.speedUp1;

                    lightStripGun.setPower(allianceColor);
                } else {
                    motorFlywheel.setPower(0);
                    flywheelState = Flywheel.off;
                    lightStripGun.setPower(0);
                }
            }
            triggered = true;
        } else {
            triggered = false;
        }

        switch (flywheelState) {
            case off:
                break;
            case speedUp1:
                if (gunTimer.milliseconds()>1200){
                    motorFlywheel.setPower(-0.64);
                    flywheelState = Flywheel.speedUp2;
                }
                break;
            case speedUp2:
                if (gunTimer.milliseconds()>2000) {
                    flywheelState = Flywheel.readyToShoot;
                }
                break;
            case readyToShoot:
                break;

        }
    }
}
