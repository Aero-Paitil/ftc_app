package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * This OpMode is used test sensor readings, no motor motion is performed. SENSOR CALIBRATIONS.
 */
public class SensorReadingOpMode extends OpMode {
    ColorSensor colorSensorDrive;
    ColorSensor colorSensorBeacon;
    //OpticalDistanceSensor opticalDistSensorLeft;
    //OpticalDistanceSensor opticalDistSensorRight;
    UltrasonicSensor ultrasonicSensor;

    @Override
    public void init() {
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        colorSensorBeacon.setI2cAddress(0x3e);
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("Distance Sensor");


        //opticalDistSensorLeft = hardwareMap.opticalDistanceSensor.get("Optical Distance Left");
        //opticalDistSensorRight = hardwareMap.opticalDistanceSensor.get("Optical Distance Right");
        //colorSensorBeacon.enableLed(false);           // disable the LED light
        //colorSensorDrive.enableLed(true);             //
    }

    @Override
    public void loop() {
        telemetry.addData("Beacon", "" + colorSensorBeacon.red() + "/" + colorSensorBeacon.green() + "/" +
                        colorSensorBeacon.blue() + "   " + colorSensorBeacon.alpha() +
                " at " +colorSensorBeacon.getI2cAddress() + " "+colorSensorBeacon.getConnectionInfo());
        telemetry.addData("Drive", "" + colorSensorDrive.red() + "/" + colorSensorDrive.green() + "/" +
                        colorSensorDrive.blue()+"   " + colorSensorDrive.alpha()+
                " at "+colorSensorDrive.getI2cAddress() + " "+colorSensorDrive.getConnectionInfo());
        telemetry.addData("Ultrasonic", ultrasonicSensor.getUltrasonicLevel());
        //telemetry.addData("OpticalDistLeft", "" + opticalDistSensorLeft.getLightDetectedRaw());
        //telemetry.addData("OpticalDistRight", "" + opticalDistSensorRight.getLightDetectedRaw());


    }
}
