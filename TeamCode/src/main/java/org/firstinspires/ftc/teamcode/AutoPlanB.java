package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;

public abstract class AutoPlanB extends LinearOpMode {

    private double DRIVING_POWER = 0.3;

    private final static int ENCODER_COUNTS_PER_ROTATION = 2 * 1140;
    private final static int SHOOT_DISTANCE1 = 21; //Drive 21 inches straight towards center before shoot
    private final static int SHOOT_STARTBELT_DISTANCE = 15;

    private boolean isBlue = isBlueAlliance();

    private static final String TAG = "Autonomous Mode";

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private ColorSensor colorSensor3c;
    private ColorSensor colorSensor3a;
    private OpticalDistanceSensor opticalSensor;

    //defining the 4 motors
    // the motors are slaved:
    // the power that goes to the first goes to the second
    private DcMotor motorLeft1;
    private DcMotor motorRight1;
    private DcMotor motorBelt;
    private Servo servoBeaconPad;
    private ModernRoboticsI2cGyro gyro = null;
    private DcMotor motorFlywheelRight;

    private double[] robotLocation = null;

    abstract boolean isBlueAlliance();

    @Override
    public void runOpMode() throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("Gyro Sensor ");
        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        //"initializing" the motors
        motorLeft1 = hardwareMap.dcMotor.get("D1left");
        //motorLeft2 = hardwareMap.dcMotor.get("D2left");
        motorRight1 = hardwareMap.dcMotor.get("D1right");
        //motorRight2 = hardwareMap.dcMotor.get("D2right");
        motorBelt = hardwareMap.dcMotor.get("Belt");
        motorFlywheelRight = hardwareMap.dcMotor.get("GunRight");
        // run flywheels by speed
        motorFlywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reset encoders
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // use running by speed mode (to make robot move straight, when equal power is applied left and right)
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range Sensor");
        colorSensor3a = hardwareMap.colorSensor.get("Color Sensor 3a");
        colorSensor3a.setI2cAddress(I2cAddr.create8bit(0x3a));
        colorSensor3a.enableLed(false);
        colorSensor3c = hardwareMap.colorSensor.get("Color Sensor 3c");
        colorSensor3c.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor3c.enableLed(false);
        opticalSensor = hardwareMap.opticalDistanceSensor.get("Optical");

        //vuforiaInit();

        gyro.resetZAxisIntegrator(); //reset gyro heading to 0
        powerMotors(0, 0);

        //waitForStart();
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry();
            /*
            telemetry.addData(">", "Robot Raw Heading = %d", getGyroRawHeading());
            telemetry.addData("optical light ", opticalSensor.getLightDetected());
            telemetry.addData("optical rawlight", opticalSensor.getRawLightDetected());
            telemetry.addData("optical max", opticalSensor.getRawLightDetectedMax());
            //telemetry.addData("bottom alpha", colorSensorBottom.alpha());
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            */
            telemetry.update();
            idle();
        }

        // if we hit stop after init, nothing else should happen
        if (!opModeIsActive() ){
            return;
        }

        gyro.resetZAxisIntegrator(); //reset gyro heading to 0
        sleep(50);

        //motorBelt.setPower(0.5); // the box is too small at the moment
        servoBeaconPad = hardwareMap.servo.get("BeaconPad");
        setPadPosition(15); // to avoid pad covering camera

        //start shooting at the position1 (3rd tile from the corner)
        moveOnShooting(-1);  //forward, -1 is a sign

        sleep(5000);
        motorBelt.setPower(0);

        moveOnShooting(1);  //backward, 1 is sign.  Backward to original place and start the beacon heading

    }


    private void setPadPosition(int pos) {
        servoBeaconPad.setPosition(pos / 255.0);
    } // from 0 to 255

    public void telemetry() {
        //telemetry.addData("Count: ", motorRight1.getCurrentPosition());
        telemetry.addData("Color 3c - R/G/B ", colorSensor3c.red() + "/" + colorSensor3c.green() + "/" +
                colorSensor3c.blue());
        telemetry.addData("Color 3a - R/G/B ", colorSensor3a.red() + "/" + colorSensor3a.green() + "/" +
                colorSensor3a.blue());
        telemetry.addData("Optical Light ",  "%.4f", opticalSensor.getLightDetected());
        telemetry.addData("Gyro Reading " , getGyroRawHeading());
        telemetry.addData("Distance ", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        if (robotLocation != null && robotLocation.length == 3) {
            telemetry.addData("Location", "x = %.0f, y = %.0f, z = %.0f", robotLocation[0], robotLocation[1], robotLocation[2]);
        }
        telemetry.update();
    }

    private void powerMotors(double leftForward, double rightForward) throws InterruptedException {
        // the left side direction is reversed
        motorLeft1.setPower(-leftForward);
        motorRight1.setPower(rightForward);
        idle();
    }

    private void powerFlywheels(boolean doPower) throws InterruptedException {
        // theflywheels should move out in the opposite direction
        if (doPower) {
            motorFlywheelRight.setPower(-1);
        } else {
            motorFlywheelRight.setPower(0);
        }
        idle();
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        motorLeft1.setMode(runMode);
        motorRight1.setMode(runMode);
    }

    private void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        motorLeft1.setZeroPowerBehavior(behavior);
        motorRight1.setZeroPowerBehavior(behavior);
    }



    //getIntegratedZValue is positive when moving ccw. We want it to behave the same way as getGyroHeading, so we changed the sign.
    private int getGyroRawHeading() {
        int heading = -gyro.getIntegratedZValue();
        telemetry.addData("Raw heading", heading);
        return heading;
    }

    private void moveOnShooting(int sign) {

        try {
            //this.powerMotors(sign * DRIVING_POWER, sign * DRIVING_POWER);
            int currentPosition;
            int startPosition = (currentPosition = motorLeft1.getCurrentPosition());
            //start belt turning at 15 inch
            double beltCount = ENCODER_COUNTS_PER_ROTATION*SHOOT_STARTBELT_DISTANCE/26.5;

            if(sign == -1){   //forward
                powerFlywheels(true);
            } else{
                powerFlywheels(false);
            }
            sleep(1000);
            if (sign == -1) {
                //while ((Math.abs(currentPosition - startPosition)) < ENCODER_COUNTS_PER_ROTATION * (SHOOT_DISTANCE1) / 26.5) {
                    //if (Math.abs(currentPosition - startPosition) > beltCount &&  sign == -1) {
                        motorBelt.setPower(1);
                    //}
                    //currentPosition = motorLeft1.getCurrentPosition();
                    idle();
                //}
            }
            //powerMotors(0,0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
