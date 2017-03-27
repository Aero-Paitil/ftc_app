package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
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
    BufferedWriter textFile;
    StringBuffer line = new StringBuffer();
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
            textFile = new BufferedWriter(new FileWriter("follow_beacon")); //DATE TIME
        } catch (IOException e) {
            e.printStackTrace();
        }
        line.append("3a Red // 3a Blue // 3c Red // 3c Blue\\n");
    }
    @Override
    public void loop() {
        int rangeFromBeacon = (int) range.getDistance(DistanceUnit.CM);
        if (rangeFromBeacon < 10) {
            drive(0, 0);
        } else if (color3a.blue() > color3a.red() && color3c.blue() > color3c.red()) {
            drive(DRIVE_POWER, DRIVE_POWER);
            writeToLine();
        } else if (color3a.blue() > color3a.red() && color3c.blue() < color3c.red()) {
            drive(DRIVE_POWER, 0);
            writeToLine();
        } else if (color3a.blue() < color3a.red() && color3c.blue() > color3c.red()) {
            drive(0, DRIVE_POWER);
            writeToLine();
        }
    }
    @Override
    public void stop() { //write to file
        drive(0, 0);
        try {
            textFile.write(line.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    void drive(double powerLeft, double powerRight) {
        motorLeft.setPower(-powerLeft);
        motorRight.setPower(-powerRight);
    }
    void writeToLine() {
        double red3a = color3a.red();
        double blue3a = color3a.blue();
        double red3c = color3c.red();
        double blue3c = color3c.blue();
        try {
            line.append(red3a + " // " + blue3a + " // " + red3c + " // " + blue3c + "\n");
        } finally {
            if (textFile != null) {
                try {
                    textFile.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}

