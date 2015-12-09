package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * This OpMode is used test sensor readings, no motor motion is performed. SENSOR CALIBRATIONS.
 */
public class SensorReadingOpMode extends OpMode {
    ColorSensor colorSensorDrive;
    //ColorSensor colorSensorBeacon;
    OpticalDistanceSensor opticalDistSensorLeft;
    OpticalDistanceSensor opticalDistSensorRight;

    @Override
    public void init() {
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        //colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        opticalDistSensorLeft = hardwareMap.opticalDistanceSensor.get("Optical Distance Left");
        opticalDistSensorRight = hardwareMap.opticalDistanceSensor.get("Optical Distance Right");
        //colorSensorBeacon.enableLed(false);           // disable the LED light
    }

    @Override
    public void loop() {
        //telemetry.addData("BeaconColor", "" + colorSensorBeacon.red() + "/" + colorSensorBeacon.green() + "/" + colorSensorBeacon.blue()
        //        + "   " + colorSensorBeacon.alpha());
        telemetry.addData("DriveAlpha", "" + colorSensorDrive.red() + "/" + colorSensorDrive.green() + "/" + colorSensorDrive.blue()
                + "   " + colorSensorDrive.alpha());
        telemetry.addData("OpticalDistLeft", "" + opticalDistSensorLeft.getLightDetectedRaw());
        telemetry.addData("OpticalDistRight", "" + opticalDistSensorRight.getLightDetectedRaw());
    }
}
