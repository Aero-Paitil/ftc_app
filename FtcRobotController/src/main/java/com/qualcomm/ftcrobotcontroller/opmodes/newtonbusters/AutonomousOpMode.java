package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Aryoman on 12/2/2015.
 */
public class AutonomousOpMode extends LinearOpMode {


    final static int STOP_AT_DISTANCE_READING = 50;                     //ODS reading; robot will stop at this distance from wall
    final static int MID_POINT_ALPHA = 20;                              //(0 + 40) / 2 = 20
    final static double CORRECTION_MULTIPLIER = 0.012;                  //kp konstant power
    final static double LINE_FOLLOW_MOTOR_POWER = -0.2;                 //power of the motor

    final static int ENCODER_COUNTS_PER_ROTATION = 2*1140;
    //one wheel rotation covers ~12 inches
    //one tile is ~24 inches long
    final static int TARGET_POSITION1 = 2*ENCODER_COUNTS_PER_ROTATION;
    final static double DRIVING_POWER = -0.5;

    MecanumWheels mecanumWheels;
    Arm arm;
    Brushes brushes;
    GyroSensor sensorGyro;

    Servo beacon6;

    ColorSensor colorSensorDrive;
    ColorSensor colorSensorBeacon;
    OpticalDistanceSensor opticalDistSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        mecanumWheels = new MecanumWheels(hardwareMap, telemetry, true);
        mecanumWheels.resetEncoders();

        brushes = new Brushes(hardwareMap, telemetry);

        arm = new Arm(hardwareMap, telemetry);

        beacon6 = hardwareMap.servo.get("Beacon6");
        beacon6.setPosition(0.5);

        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        opticalDistSensor = hardwareMap.opticalDistanceSensor.get("Optical Distance");

        waitForStart();

        //our robot is faced backwards

        //assuming we are starting at the tile next to the mountain,

        //go backwards 2*24 inches
        mecanumWheels.runToPosition(TARGET_POSITION1, DRIVING_POWER);

        //release brushes
        brushes.autonomousUndockBrushes();

        //undock arm
        arm.autonomousUndockArm();

        //rotate robot 45 degrees clockwise
        mecanumWheels.rotate(45);

        //todo: drive backwards until the robot runs into the white line leading to the beacon.
    }
}
