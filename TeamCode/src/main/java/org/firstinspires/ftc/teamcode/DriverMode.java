package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverMode", group="nb")
//@Disabled

public class DriverMode extends OpMode {

    //public static final String ALLIANCE = "RED";
//    private ElapsedTime timer = new ElapsedTime();
//    private ElapsedTime motorleftTimer = new ElapsedTime();
//    private ElapsedTime motorrightTimer = new ElapsedTime();
//    private double lastLeftForward = 0;
//    private double lastRightForward = 0;

//    private static final String TAG = "Driver Mode";

    //defining the 4 motors
    private DcMotor motorLeft1;
    private DcMotor motorRight1;
    private DcMotor motorLeft2;
    private DcMotor motorRight2;
    private DcMotor motorBrush;
    private boolean inBrushBtnPressed = false;
    private boolean outBrushBtnPressed = false;
    private int brushState = 0; //-1:Intake, 1:Sweep out, 0:Stopped

    private boolean forward;
    private boolean backButtonPressed = false;

    private static double MINPOWER = 0.15;

    private double scaled(double x){
        return 0.5*(x/1.07)*(.62*x*x + .45);
    }

    @Override
    public void init() {
        //"initializing" the motors
        motorLeft1 = hardwareMap.dcMotor.get("D1left");
        motorRight1 = hardwareMap.dcMotor.get("D1right");
        motorLeft2 = hardwareMap.dcMotor.get("D2left");
        motorRight2 = hardwareMap.dcMotor.get("D2right");
        motorBrush = hardwareMap.dcMotor.get("Brush");

        //setting the motors on the right side in reverse so both wheels spin the same way.
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        forward = true;

    }

    @Override
    public void loop() {

        //Changing Brush State
        if (gamepad1.x){
            inBrushBtnPressed = true;
        } else if (gamepad1.b){
            outBrushBtnPressed = true;
        }else{
            if (inBrushBtnPressed){
                inBrushBtnPressed = false;
                brushState = (brushState==0) ? -1 : 0;
            }
            else if (outBrushBtnPressed) {
                outBrushBtnPressed = false;
                brushState = (brushState == 0) ? 1 : 0;
            }
        }
        if (brushState == -1){
            motorBrush.setPower(1);
            telemetry.addData("Brush", "Intake");
        }
        else if (brushState == 1){
            motorBrush.setPower(-1);
            telemetry.addData("Brush", "Sweep Out");
        } else{
            motorBrush.setPower(0);
            telemetry.addData("Brush", "Stopped");
        }

        double leftForward;
        double rightForward;

        int sign = forward ? 1 : -1;
        if (!gamepad1.left_bumper && !gamepad1.right_bumper){
            //Arcade Drive
            rightForward = -(scaled(gamepad1.left_stick_y) + sign*scaled(gamepad1.left_stick_x));
            leftForward = -(scaled(gamepad1.left_stick_y) - sign*scaled(gamepad1.left_stick_x));
        }
        else {
            //Tank Drive
            leftForward = -scaled(gamepad1.left_stick_y);
            rightForward = -scaled(gamepad1.right_stick_y);
        }

        if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)) {
            // dPadDrive: Forward, Backwards, Rotate in place CClockwise and Clockwise
            //We are using robot coordinates
            double dpadSpeed = 0.2;
            if (gamepad1.dpad_up) {
                rightForward = dpadSpeed;
                leftForward = dpadSpeed;
            } else if (gamepad1.dpad_down) {
                rightForward = -dpadSpeed;
                leftForward = -dpadSpeed;
            } else if (gamepad1.dpad_left) {
                rightForward = sign*dpadSpeed;
                leftForward = -sign*dpadSpeed;
            } else {
                leftForward = sign*dpadSpeed;
                rightForward = -sign*dpadSpeed;
            }
        }


        //driving backwards
        if (!forward){
            leftForward = -leftForward;
            rightForward = -rightForward;
        }

        //todo adjust the deadband
        if ((rightForward > -0.01) && (rightForward < 0.01))
            rightForward = 0;
        else if ((rightForward > -MINPOWER) && (rightForward < MINPOWER))
            rightForward = MINPOWER * rightForward/Math.abs(rightForward);

        if ((leftForward > -0.01) && (leftForward < 0.01))
            leftForward = 0;
        else if ((leftForward > -MINPOWER) && (leftForward < MINPOWER))
            leftForward = MINPOWER * leftForward/Math.abs(leftForward);

        rightForward = Range.clip(rightForward, -1, 1);
        leftForward = Range.clip(leftForward, -1, 1);

        //Changing direction from forward to backward and backward to forward
        //Gamepad back button does not work with motorola 3G
        if (gamepad1.y) {
            backButtonPressed = true;
        } else if (backButtonPressed) {
            backButtonPressed = false;
            forward = !forward;
        }


        telemetry.addData("left",leftForward);
        telemetry.addData("right",rightForward);

        telemetry.addData("back(Y) button pressed", backButtonPressed);
        telemetry.addData("forward", forward);


        /* assigning the motors the scaled powers that we just calculated in the step above. */
        powerMotors(rightForward, leftForward);

    }

    private void powerMotors(double rightForward, double leftForward) {
        motorLeft1.setPower(leftForward);
        motorLeft2.setPower(leftForward);
        motorRight1.setPower(rightForward);
        motorRight2.setPower(rightForward);
    }


}