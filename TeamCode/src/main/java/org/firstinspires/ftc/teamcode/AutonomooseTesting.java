package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

/**
 * Created by Brandon Lee on 10/30/2016.
 * Updated by Athena 10/30/2016 - rotate to find an image and detect robot location
 * Updated by Athena 11/05/2016 - rotate to an angle
 * Updated by Sangmin and Jasmine 11/05/2016 - drive straight using gyro
 */

@Autonomous(name="TheMooseTest", group="nb")
public class AutonomooseTesting extends AutonomousMode {

    @Override
    boolean isBlueAlliance() {
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TESTING = true;
        super.runOpMode();
    }

    private int idx = 0;

    @Override
    protected void testSequence() throws InterruptedException {
        //testMoveByInchesGyro(0.03, -0.92, 0, 32, -0.3);
        int head = 10; //20; //8;
        double speed = 0.92;
        double finalspeed = 0.3;

        idx = 0;
        testMoveByInchesGyro(0.02, speed, head, 56, finalspeed);
        testMoveByInchesGyro(0.02, -speed, -head, 56, -finalspeed);

        idx = 0;
        testMoveByInchesGyro(0.03, speed, head, 56, finalspeed);
        testMoveByInchesGyro(0.03, -speed, -head, 56, -finalspeed);

        idx = 0;
        testMoveByInchesGyro(0.04, speed, head, 56, finalspeed);
        testMoveByInchesGyro(0.04, -speed, -head, 56, -finalspeed);

        idx = 0;
        testMoveByInchesGyro(0.05, speed, head, 56, finalspeed);
        testMoveByInchesGyro(0.05, -speed, -head, 56, -finalspeed);


        //testOpticalDistanceSensor();
        //movePartCircle(1.5 * TILE_LENGTH, 1, false, 0.6);

    }

    private void moveByInchesGyroTest() throws InterruptedException {
        out.append("Moving from -0.92 power to -0.3").append("\n");
        moveByInchesGyro(-0.92, 0, 30, -0.3);
        powerMotors(0, 0);
        sleep(2000);
        out.append("Moving from 0.92 power to 0.3").append("\n");
        moveByInchesGyro(0.92, 0, 30, 0.3);
        powerMotors(0, 0);
        sleep(2000);
        out.append("Moving from -0.3 power to -0.92").append("\n");
        moveByInchesGyro(-0.3, 0, 30, -0.92);
        powerMotors(0, 0);
        sleep(2000);
        out.append("Moving from 0.3 power to 0.92").append("\n");
        moveByInchesGyro(0.3, 0, 30, 0.92);
        powerMotors(0, 0);
    }

    private void testOpticalDistanceSensor() throws InterruptedException {
        int delayMs = 500;

        double inches = 3;
        for (int i = 0; i < 4; i++) {
            gyro.resetZAxisIntegrator();
            rotateOverLine(inches, 0.3);
            sleep(delayMs);
            gyro.resetZAxisIntegrator(); //reset gyro heading to 0
            sleep(1000);
            inches = inches + 2;
            rotateOverLine(-(inches), 0.3);
            sleep(delayMs);
            inches = inches + 2;
        }
    }




    /**
     * If inches are positive the robot is rotating counterclockwise
     * @param inches inches to go
     * @param drivingPower driving power, always positive
     * @throws InterruptedException
     */
    private void rotateOverLine(double inches, double drivingPower) throws InterruptedException{ // moves 26.5 in one rotation

        int counts = motorRight1.getCurrentPosition();
        double sign = Math.round(inches/Math.abs(inches));
        // positive sign - counterclockwise
        //powerMotors(sign*drivingPower, -sign*drivingPower);
        // for now - no rotate just go straight over the line
        powerMotors(sign*drivingPower, sign*drivingPower);
        int counter = 0;
        int iters = 0;
        long cms;
        long maxdiff = 0;
        long sumdiff = 0;
        long firsttime = -1;
        long endtime;
        ArrayList<Integer> position = new ArrayList<>();
        ArrayList<Integer> heading = new ArrayList<>();
        ArrayList<Double> detected = new ArrayList<>();
        ArrayList<Long> currentTimeMs = new ArrayList<>();
        double lightDetected;
        int currPos = Math.abs(motorRight1.getCurrentPosition() - counts);
        long initialMs = System.currentTimeMillis();
        long ms = initialMs;
        long whitetime=0;
        while (opModeIsActive() && currPos < Math.abs(inchesToCounts(inches))) {
            position.add(currPos);
            cms = System.currentTimeMillis();
            currentTimeMs.add(cms-initialMs);
            lightDetected = opticalSensor.getLightDetected();
            detected.add(lightDetected);
            heading.add(getGyroRawHeading());
            if ( lightDetected > MID_POINT_LIGHT_BACK){
                counter++;
                if (firsttime == -1){
                    firsttime = System.currentTimeMillis();
                }
            } else {
                if (whitetime == 0 && firsttime > 0){
                    endtime = System.currentTimeMillis();
                    whitetime = endtime-firsttime;
                }
            }
            sumdiff += cms - ms;
            if (cms - ms > maxdiff) maxdiff = cms - ms;
            ms = cms;
            iters++;
            currPos = Math.abs(motorRight1.getCurrentPosition() - counts);
        }
        telemetry.addData("WhiteTime: ", whitetime);
        telemetry.addData("Maxdiff: ", maxdiff);
        telemetry.addData("Avgdiff: ", (double)sumdiff / (iters));
        telemetry.addData("Counter: ", counter);
        telemetry.addData("Iterations: ", iters);

        powerMotors(0,0);

        // write to file when missed line or first time moving in the same direction after missed line

        try {
            File file = new File(Environment.getExternalStorageDirectory().getPath() + (counter == 0 ? "/FIRST/missedStraight" : "/FIRST/detectedStraight") + idx + ".txt");
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write("# WhiteTime: "+whitetime + "\n");
            outputStreamWriter.write("# Maxdiff: "+maxdiff + "\n");
            outputStreamWriter.write("# Avgdiff: "+(double)sumdiff / (iters)+ "\n");
            outputStreamWriter.write("# Counter: "+counter+ "\n");
            outputStreamWriter.write("# Iterations: "+iters+ "\n");
            outputStreamWriter.write("time,counts,gyro,ods\n");
            for (int i = 0; i < position.size(); i++) {
                outputStreamWriter.write(currentTimeMs.get(i) + "," + position.get(i) + "," + heading.get(i) +","+ detected.get(i) + "\n");
            }
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }

        telemetry.update();
        idx++;
    }


    boolean testMoveByInchesGyro(double kp, double startPower, int heading, double maxInches, double endPower) throws InterruptedException {
        ArrayList<Double> errors = new ArrayList<>();
        ArrayList<Double> currentTimeMs = new ArrayList<>();

        int initialcount = motorLeft1.getCurrentPosition();
        double error, steerSpeed; // counterclockwise speed
        double inchesForGradient = 10;
       // double kp = 0.03; //experimental coefficient for proportional correction of the direction
        double countsSinceStart = Math.abs(motorLeft1.getCurrentPosition() - initialcount);
        double slope = (endPower - startPower) / inchesToCounts(Math.min(inchesForGradient, maxInches)); // this slope is for calculating power
        double countsForGradient = (maxInches < inchesForGradient) ? 0 : Math.abs(inchesToCounts(maxInches - inchesForGradient));
        double motorPower = startPower;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && countsSinceStart < inchesToCounts(maxInches)) {
            // error CCW - negative, CW - positive
            error = getRawHeadingError(heading);
            errors.add(error);
            currentTimeMs.add(timer.milliseconds());
            steerSpeed = kp * error / 4;
            telemetry.addData("Error", error);
            telemetry.update();


            if (countsSinceStart > countsForGradient) {
                motorPower = slope * (countsSinceStart - inchesToCounts(maxInches)) + endPower;
            }
            powerMotors(Range.clip(motorPower - steerSpeed, -1.0, 1.0), Range.clip(motorPower + steerSpeed, -1.0, 1.0));

            countsSinceStart = Math.abs(motorLeft1.getCurrentPosition() - initialcount);
        }
        powerMotors(0,0);

        try {
            idx++;
            File file = new File(Environment.getExternalStorageDirectory().getPath() +  "/FIRST/gyrocorrect"+kp+"-"+Math.abs(heading)+"-"+idx);
            telemetry.addData("File", file.getAbsolutePath());

            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(file));
            outputStreamWriter.write("time,error\n");
            for (int i = 0; i < errors.size(); i++) {
                outputStreamWriter.write(currentTimeMs.get(i) + "," + errors.get(i) + "\n");
            }
            outputStreamWriter.close();

        } catch (Exception e) {
            telemetry.addData("Exception", "File write failed: " + e.toString());
        }
        sleep(2000);
        gyro.resetZAxisIntegrator(); //reset gyro heading to 0
        sleep(50);
        return opModeIsActive();
    }

}
