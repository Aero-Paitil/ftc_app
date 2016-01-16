package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * This OpMode is used test sensor readings, no motor motion is performed. SENSOR CALIBRATIONS.
 */
public class SensorReadingOpMode extends OpMode {
    ColorSensor colorSensorFront;
    ColorSensor colorSensorDrive;
    ColorSensor colorSensorBeacon;
    //OpticalDistanceSensor opticalDistSensorLeft;
    //OpticalDistanceSensor opticalDistSensorRight;
    UltrasonicSensor ultrasonicSensorRight;
    UltrasonicSensor ultrasonicSensorLeft;

    @Override
    public void init() {

        colorSensorFront = hardwareMap.colorSensor.get("Color Sensor Front");
        colorSensorFront.setI2cAddress(0x40);
        colorSensorFront.enableLed(true);
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorDrive.enableLed(true);
        colorSensorBeacon = hardwareMap.colorSensor.get("Color Sensor Beacon");
        colorSensorBeacon.setI2cAddress(0x3e);
        ultrasonicSensorRight = hardwareMap.ultrasonicSensor.get("Distance Sensor Right");
        ultrasonicSensorLeft = hardwareMap.ultrasonicSensor.get("Distance Sensor Left");

    }

    @Override
    public void loop() {
        telemetry.addData("Beacon", "" + colorSensorBeacon.red() + "/" + colorSensorBeacon.green() + "/" +
                colorSensorBeacon.blue() + "   " + colorSensorBeacon.alpha() +
                " at " +colorSensorBeacon.getI2cAddress() + " "+colorSensorBeacon.getConnectionInfo());
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


    }
}