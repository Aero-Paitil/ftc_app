package org.firstinspires.ftc.teamcode.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * This OpMode is used test sensor readings, no motor motion is performed. SENSOR CALIBRATIONS.
 */

@TeleOp(name="Sensor Reading", group="Main")

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
        colorSensorFront.setI2cAddress(new I2cAddr(0x40));
        colorSensorFront.enableLed(true);
        colorSensorDrive = hardwareMap.colorSensor.get("Color Sensor Bottom");
        colorSensorDrive.enableLed(true);
        colorSensorBeaconRight = hardwareMap.colorSensor.get("Color Sensor Beacon Right");
        colorSensorBeaconRight.setI2cAddress(new I2cAddr(0x3e));
        colorSensorBeaconRight.enableLed(false);
        colorSensorBeaconLeft = hardwareMap.colorSensor.get("Color Sensor Beacon Left");
        colorSensorBeaconLeft.setI2cAddress(new I2cAddr(0x42));
        colorSensorBeaconLeft.enableLed(false);
        ultrasonicSensorRight = hardwareMap.ultrasonicSensor.get("Distance Sensor Right");
        ultrasonicSensorLeft = hardwareMap.ultrasonicSensor.get("Distance Sensor Left");
        touchSensor = hardwareMap.touchSensor.get("Rear Wheels Touch");
        opticalDistance = hardwareMap.opticalDistanceSensor.get("Optical Distance");


    }

    @Override
    public void loop() {
        telemetry.addData("Beacon Right - R/G/B: ", "" + colorSensorBeaconRight.red() + "/" + colorSensorBeaconRight.green() + "/" +
                colorSensorBeaconRight.blue() + "       Light reading:" + colorSensorBeaconRight.alpha() +
                " at " + colorSensorBeaconRight.getI2cAddress() + " " + colorSensorBeaconRight.getConnectionInfo());
        telemetry.addData("Beacon Left - R/G/B: ", "" + colorSensorBeaconLeft.red() + "/" + colorSensorBeaconLeft.green() + "/" +
                colorSensorBeaconLeft.blue() + "       Light reading:" + colorSensorBeaconLeft.alpha() +
                " at " + colorSensorBeaconLeft.getI2cAddress() + " " + colorSensorBeaconLeft.getConnectionInfo());
        telemetry.addData("Drive Back - R/G/B: ", "" + colorSensorDrive.red() + "/" + colorSensorDrive.green() + "/" +
                colorSensorDrive.blue()+"       Light reading:" + colorSensorDrive.alpha()+
                " at "+colorSensorDrive.getI2cAddress() + " "+colorSensorDrive.getConnectionInfo());
        telemetry.addData("Drive Front - R/G/B: ", "" + colorSensorFront.red() + "/" + colorSensorFront.green() + "/" +
                colorSensorFront.blue()+"       Light reading:" + colorSensorFront.alpha()+
                " at "+colorSensorFront.getI2cAddress() + " "+colorSensorFront.getConnectionInfo());
        telemetry.addData("Ultrasonic Right", ultrasonicSensorRight.getUltrasonicLevel());
        telemetry.addData("Ultrasonic Left", ultrasonicSensorLeft.getUltrasonicLevel());
        //telemetry.addData("OpticalDistLeft", "" + opticalDistSensorLeft.getLightDetectedRaw());
        //telemetry.addData("OpticalDistRight", "" + opticalDistSensorRight.getLightDetectedRaw());
        telemetry.addData("Optical Distance",opticalDistance.getRawLightDetected());
        telemetry.addData("Touch Sensor", touchSensor.isPressed());


    }
}