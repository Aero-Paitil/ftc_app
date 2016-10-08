package org.firstinspires.ftc.teamcode.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Allison 11/20/15
 * Modified by Jasmine and Simone 11/21/15
 * -added code to move brush handles and turn brushes
 * Modified by Athena 11/22/15
 * -fixed undockBrushes and dockBrushes methods; added constants for positions and controls to start/stop brush rotation.
 * !!BRUSHES SHOULD BE DOCKED RIGHT FIRST THEN LEFT!!
 * To complete action, hold down dpad until action is complete OR press same control/button multiple times
 */



public class BrushOpMode extends OpMode {

   Brushes brushes;

    @Override
    public void init() {
        brushes = new Brushes (hardwareMap ,telemetry);
    }



    @Override
    public void loop() {
       brushes.printTelemetry();

        if (gamepad2.dpad_up) {
            brushes.undockBrushes();
        } else if (gamepad2.dpad_down) {
            brushes.dockBrushes();
        } else if (gamepad2.dpad_right) {
            //rotate brushes inward
            brushes.setRotation(Brushes.ROTATE_INWARD_FAST);
        } else if (gamepad2.dpad_left) {
            //rotate brushes outward
            brushes.setRotation(Brushes.ROTATE_OUTWARD_FAST);
        } else if (gamepad2.x) {
            //stop the movement of the brushes
            brushes.setRotation(Brushes.STOP_ROTATION);
        }

        /*
        //todo add ability to undock one handle
        // when a button is pressed, left trigger position controls the handle position
        // not pressed trigger means handle is out
        // pressed all the way trigger means handles in
        if (gamepad2.a){
            leftBrushHandle.setPosition(1-gamepad2.left_trigger);
            rightBrushHandle.setPosition(1-gamepad2.right_trigger);
        }
        */

    }
}