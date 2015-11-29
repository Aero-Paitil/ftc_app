package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Athena Zheng on 11/28/2015.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class PushbotFollowLine extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    ColorSensor colorSensor;
    long startTime;
    //OpticalDistanceSensor ods_sensor;

    //Robot States
    enum State {Drive, BackUp, Turn}
    State state;

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void init() {
        // Get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        // Reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Get a reference to the color sensor
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        //ods_sensor = hardwareMap.opticalDistanceSensor.get("ods_sensor");

        //set the state
        state = State.Drive;
    }

    @Override
    public void loop() {
        boolean isWhiteLight=false;
        double Kp = -0.7;                                // REMEMBER we are using Kp*100 so this is really 10 !
        int offset = 16;                                 //Initialize the other two variables
        double Tp = -0.2;

        //get the color detected, assign to variable
        int colorHue = colorSensor.alpha(); //white tape leading to beacon
        //double lightReading = ods_sensor.getLightDetected();
        //isWhiteLight = lightReading > 0.8 ? true: false;

        //when isWhiteLight is true, follow the line
        //if(isWhiteLight){
        if (System.currentTimeMillis() - startTime < 5000) {

            // calculate the error by subtracting the offset
            double error = colorHue - offset;
            // the "P term", how much we want to change the motors' power
            double turn = Kp * error;
            //REMEMBER to undo the affect of the factor of 100 in Kp !
            turn = turn/100;
            leftMotor.setPower(Tp + turn);                  // the power level for the left motor
            rightMotor.setPower(Tp - turn);                 //the power level for the right motor
        }
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        /*switch(state) {
            case Drive:
                if (colorSensor.equals(colorHue)) { //if color sensor detects the hue of the tape, then drive forward slowly
                    leftMotor.setPower(0.15);
                    rightMotor.setPower(0);
                } else { //if color sensor does NOT detect the hue of the tape, then switch state to Turn
                    state = State.Turn;
                }
                break;
            case Turn:
                leftMotor.setPower(0);
                rightMotor.setPower(0.15);
                if (colorSensor.equals(colorHue)) { //if color sensor detects the hue of the tape, then drive forward slowly
                    state = State.Drive;
                }
                break;
        } */

        telemetry.addData("Hue", colorHue);
    }
}
