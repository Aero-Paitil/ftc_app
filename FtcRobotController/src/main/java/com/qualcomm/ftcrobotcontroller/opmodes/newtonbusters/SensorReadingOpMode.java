package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * This OpMode is used test sensor readings, no motor motion is performed. SENSOR CALIBRATIONS.
 */
public class SensorReadingOpMode extends OpMode {
    ColorSensor colorSensorFront;
    ColorSensor colorSensorDrive;
    ColorSensor colorSensorBeaconRight;
    ColorSensor colorSensorBeaconLeft;
    //OpticalDistanceSensor opticalDistSensorLeft;
    //OpticalDistanceSensor opticalDistSensorRight;
    UltrasonicSensor ultrasonicSensorRight;
    UltrasonicSensor ultrasonicSensorLeft;
    TouchSensor touchSensor;
    OpticalDistanceSensor opticalDistance;
    @Override
    public void init() {

        colorSensorFront = hardwareMap.colorSensor.get("Color Sensor Front");
        colorSensorFront.setI2cAddress(0x40);
        colorSensorFront.enableLed(true);
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorDrive.enableLed(true);
        colorSensorBeaconRight = hardwareMap.colorSensor.get("Color Sensor Beacon Right");
        colorSensorBeaconRight.setI2cAddress(0x3e);
        colorSensorBeaconRight.enableLed(false);
        colorSensorBeaconLeft = hardwareMap.colorSensor.get("Color Sensor Beacon Left");
        colorSensorBeaconLeft.setI2cAddress(0x42);
        colorSensorBeaconLeft.enableLed(false);
        ultrasonicSensorRight = hardwareMap.ultrasonicSensor.get("Distance Sensor Right");
        ultrasonicSensorLeft = hardwareMap.ultrasonicSensor.get("Distance Sensor Left");
        touchSensor = hardwareMap.touchSensor.get("Rear Wheels Touch");
        opticalDistance = hardwareMap.opticalDistanceSensor.get("Optical Distance");


    }

    @Override
    public void loop() {
        telemetry.addData("Beacon Right", "" + colorSensorBeaconRight.red() + "/" + colorSensorBeaconRight.green() + "/" +
                colorSensorBeaconRight.blue() + "   " + colorSensorBeaconRight.alpha() +
                " at " + colorSensorBeaconRight.getI2cAddress() + " " + colorSensorBeaconRight.getConnectionInfo());
        telemetry.addData("Beacon Left", "" + colorSensorBeaconLeft.red() + "/" + colorSensorBeaconLeft.green() + "/" +
                colorSensorBeaconLeft.blue() + "   " + colorSensorBeaconLeft.alpha() +
                " at " + colorSensorBeaconLeft.getI2cAddress() + " " + colorSensorBeaconLeft.getConnectionInfo());
        telemetry.addData("Drive Back", "" + colorSensorDrive.red() + "/" + colorSensorDrive.green() + "/" +
                colorSensorDrive.blue()+"   " + colorSensorDrive.alpha()+
                " at "+colorSensorDrive.getI2cAddress() + " "+colorSensorDrive.getConnectionInfo());
        telemetry.addData("Drive Front", "" + colorSensorFront.red() + "/" + colorSensorFront.green() + "/" +
                colorSensorFront.blue()+"   " + colorSensorFront.alpha()+
                " at "+colorSensorFront.getI2cAddress() + " "+colorSensorFront.getConnectionInfo());
        telemetry.addData("Ultrasonic Right", ultrasonicSensorRight.getUltrasonicLevel());
        telemetry.addData("Ultrasonic Left", ultrasonicSensorLeft.getUltrasonicLevel());
        //telemetry.addData("OpticalDistLeft", "" + opticalDistSensorLeft.getLightDetectedRaw());
        //telemetry.addData("OpticalDistRight", "" + opticalDistSensorRight.getLightDetectedRaw());
        telemetry.addData("Optical Distance",opticalDistance.getLightDetectedRaw());
        telemetry.addData("Touch Sensor", touchSensor.isPressed());


    }
}