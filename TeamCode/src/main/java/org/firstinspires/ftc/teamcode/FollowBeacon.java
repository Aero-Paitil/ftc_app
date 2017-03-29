package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;
import java.io.BufferedWriter;
import java.io.FileWriter;

/**
 * Created by Athena on 3/25/2017.
 * will be following blue color, not red color
 */

@TeleOp(name="FollowBeacon", group="nb")

public class FollowBeacon extends OpMode {

    double DRIVE_POWER = 0.225;

    DcMotor motorLeft, motorRight;

    ColorSensor color3a, color3c;
    ModernRoboticsI2cRangeSensor range; //range 1 - 255 cm

    BufferedWriter bufferedWriter;
    StringBuffer line = new StringBuffer("3a Red // 3a Blue // 3c Red // 3c Blue\n");

    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("D1left");
        motorRight = hardwareMap.dcMotor.get("D1right");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        color3a = hardwareMap.colorSensor.get("Color Sensor 3a");
        color3a.setI2cAddress(I2cAddr.create8bit(0x3a));
        color3a.enableLed(false);
        color3c = hardwareMap.colorSensor.get("Color Sensor 3c");
        color3c.setI2cAddress(I2cAddr.create8bit(0x3c));
        color3c.enableLed(false);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range Sensor");

        try {
            bufferedWriter = new BufferedWriter(new FileWriter(Environment.getExternalStorageDirectory().getPath() + "/FIRST/followBeacon.txt")); //DATE TIME
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {
       long time = System.currentTimeMillis();
        int minRange = (int) range.getDistance(DistanceUnit.CM);
        if (minRange < 20 || color3a.blue() < 1 && color3c.blue() <1) {
            drive(0, 0);
        } else if (color3a.blue() > 0 && color3c.blue() > 0) { //(color3a.blue() > color3a.red() && color3c.blue() > color3c.red()) {
            drive(DRIVE_POWER, DRIVE_POWER);
            //while (time < 1000) {
                ;
            //}
           writeToLine();
        } else if (color3a.blue() > color3c.blue() || color3a.blue() > 0 && color3c.blue() == 0) {//(color3a.blue() > color3a.red() && color3c.blue() < color3c.red()) {
            drive(DRIVE_POWER, 0);
            //while (time < 1000) {
                ;
            //}
            writeToLine();
        } else if (color3a.blue() < color3c.blue() || color3a.blue() == 0 && color3c.blue() > 0) { //(color3a.blue() < color3a.red() && color3c.blue() > color3c.red()) {
            drive(0, DRIVE_POWER);
            //while (time < 1000) {
                ;
            //}
            writeToLine();
        }
    }

    @Override
    public void stop() { //write to file
        try {
            bufferedWriter.write(line.toString());
            bufferedWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    void drive(double powerLeft, double powerRight) {
        motorLeft.setPower(-powerLeft);
        motorRight.setPower(-powerRight);
    }

    void writeToLine() {
        line.append(color3a.red() + " // " + color3a.blue() + " // " + color3c.red() + " // " + color3c.blue() + "\n");
    }
}