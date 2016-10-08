package org.firstinspires.ftc.teamcode.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;


//created by Athena Zheng on 4/2/16

public class TestOpMode extends OpMode {

    enum CheckState {
        unChecked, checked, stuck
    }

    HashMap<String, CheckState> buttons = new HashMap<String, CheckState>();
    HashMap<String, CheckState> sticksAndTrigs = new HashMap<String, CheckState>();

    ElapsedTime tenSec;

    //gamepad1
    final String GP1_A = "gamepad1, button a";
    final String GP1_B = "gamepad1, button b";
    final String GP1_X = "gamepad1, button x";
    final String GP1_Y = "gamepad1, button y";
    final String GP1_GUIDE = "gamepad1, guide";
    final String GP1_DPAD_UP = "gamepad1, dpad up";
    final String GP1_DPAD_DOWN = "gamepad1, dpad down";
    final String GP1_DPAD_LEFT = "gamepad1, dpad left";
    final String GP1_DPAD_RIGHT = "gamepad1, dpad right";
    final String GP1_LEFT_BUMPER = "gamepad1, left bumper";
    final String GP1_RIGHT_BUMPER = "gamepad1, right bumper";
    final String GP1_LEFT_STICK_X = "gamepad1, left stick x";
    final String GP1_LEFT_STICK_Y = "gamepad1, left stick y";
    final String GP1_RIGHT_STICK_X = "gamepad1, right stick x";

    //gamepad2
    final String GP2_A = "gamepad2, button a";
    final String GP2_B = "gamepad2, button b";
    final String GP2_START = "gamepad2, start";
    final String GP2_GUIDE = "gamepad2, guide";
    final String GP2_DPAD_LEFT = "gamepad2, dpad left";
    final String GP2_DPAD_RIGHT = "gamepad2, dpad right";
    final String GP2_LEFT_TRIGGER = "gamepad2, left trigger";
    final String GP2_RIGHT_TRIGGER = "gamepad2, right trigger";
    final String GP2_RIGHT_STICK_Y = "gamepad2, right stick y";


    @Override
    public void init() {

        //gamepad1
        buttons.put(GP1_A, gamepad1.a ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_B, gamepad1.b ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_X, gamepad1.x ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_Y, gamepad1.y ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_GUIDE, gamepad1.guide ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_DPAD_UP, gamepad1.dpad_up ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_DPAD_DOWN, gamepad1.dpad_down ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_DPAD_LEFT, gamepad1.dpad_left ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_DPAD_RIGHT, gamepad1.dpad_right ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_LEFT_BUMPER, gamepad1.left_bumper ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP1_RIGHT_BUMPER, gamepad1.right_bumper ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put(GP1_LEFT_STICK_X, gamepad1.left_stick_x != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put(GP1_LEFT_STICK_Y, gamepad1.left_stick_y != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put(GP1_RIGHT_STICK_X, gamepad1.right_stick_x != 0 ? CheckState.stuck : CheckState.unChecked);

        //gamepad2
        buttons.put(GP2_A, gamepad2.a ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP2_B, gamepad2.y ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP2_START, gamepad2.start ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP2_GUIDE, gamepad2.guide ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP2_DPAD_LEFT, gamepad2.dpad_left ? CheckState.stuck : CheckState.unChecked);
        buttons.put(GP2_DPAD_RIGHT, gamepad2.dpad_right ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put(GP2_LEFT_TRIGGER, gamepad2.left_trigger != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put(GP2_RIGHT_TRIGGER, gamepad2.right_trigger != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put(GP2_RIGHT_STICK_Y, gamepad2.right_stick_y != 0 ? CheckState.stuck : CheckState.unChecked);

        telemetry.addData("For ten seconds the stuck buttons will show (mode 1), and then after that the unchecked buttons/sticks list will appear (mode 2).", "");

        tenSec = new ElapsedTime();

    }

    //step1: will print out any buttons/sticks that are considered stuck
    //step2: will print out any buttons/sticks that are unchecked

    public void loop() {

        if (tenSec.time() < 10) {

            telemetry.addData("Mode 1: Stuck buttons/sticks -- ", "");

            //gamepad1
            if (gamepad1.a) {
                telemetry.addData(GP1_A, "");
            }

            if (gamepad1.b) {
                telemetry.addData(GP1_B, "");
            }

            if (gamepad1.x) {
                telemetry.addData(GP1_X, "");
            }

            if (gamepad1.y) {
                telemetry.addData(GP1_Y, "");
            }

            if (gamepad1.guide) {
                telemetry.addData(GP1_GUIDE, "");
            }

            if (gamepad1.dpad_up) {
                telemetry.addData(GP1_DPAD_UP, "");
            }

            if (gamepad1.dpad_down) {
                telemetry.addData(GP1_DPAD_DOWN, "");
            }

            if (gamepad1.dpad_left) {
                telemetry.addData(GP1_DPAD_LEFT, "");
            }

            if (gamepad1.dpad_right) {
                telemetry.addData(GP1_DPAD_RIGHT, "");
            }

            if (gamepad1.left_bumper) {
                telemetry.addData(GP1_LEFT_BUMPER, "");
            }

            if (gamepad1.right_bumper) {
                telemetry.addData(GP1_RIGHT_BUMPER, "");
            }

            if (gamepad1.left_stick_x != 0) {
                telemetry.addData(GP1_LEFT_STICK_X, "");
            }

            if (gamepad1.left_stick_y != 0) {
                telemetry.addData(GP1_LEFT_STICK_Y, "");
            }

            if (gamepad1.right_stick_x != 0) {
                telemetry.addData(GP1_RIGHT_STICK_X, "");
            }

            //gamepad2
            if (gamepad2.a) {
                telemetry.addData(GP2_A, "");
            }

            if (gamepad2.b) {
                telemetry.addData(GP2_B, "");
            }

            if (gamepad2.start) {
                telemetry.addData("gamepad2, start button", "");
            }

            if (gamepad2.guide) {
                telemetry.addData("gamepad2, guide button", "");
            }

            if (gamepad1.dpad_left) {
                telemetry.addData(GP2_DPAD_LEFT, "");
            }

            if (gamepad1.dpad_right) {
                telemetry.addData(GP2_DPAD_RIGHT, "");
            }

            if (gamepad1.left_trigger != 0) {
                telemetry.addData(GP2_LEFT_TRIGGER, "");
            }

            if (gamepad1.right_trigger != 0) {
                telemetry.addData(GP2_RIGHT_TRIGGER, "");
            }

            if (gamepad1.right_stick_y != 0) {
                telemetry.addData("gamepad1, right stick y", "");
            }

        } else {

            telemetry.addData("Mode 2: Unchecked buttons/sticks list -- ", "");

            //gamepad1
            if (gamepad1.a) {
                if (buttons.get(GP1_A) == CheckState.unChecked) {
                    buttons.put(GP1_A, CheckState.checked);
                }
            }

            if (gamepad1.b) {
                if (buttons.get(GP1_B) == CheckState.unChecked) {
                    buttons.put(GP1_B, CheckState.checked);
                }
            }

            if (gamepad1.x) {
                if (buttons.get(GP1_X) == CheckState.unChecked) {
                    buttons.put(GP1_X, CheckState.checked);
                }
            }

            if (gamepad1.y) {
                if (buttons.get(GP1_Y) == CheckState.unChecked) {
                    buttons.put(GP1_Y, CheckState.checked);
                }
            }

            if (gamepad1.guide) {
                if (buttons.get(GP1_GUIDE) == CheckState.unChecked) {
                    buttons.put(GP1_GUIDE, CheckState.checked);
                }
            }

            if (gamepad1.dpad_up) {
                if (buttons.get(GP1_DPAD_UP) == CheckState.unChecked) {
                    buttons.put(GP1_DPAD_UP, CheckState.checked);
                }
            }

            if (gamepad1.dpad_down) {
                if (buttons.get(GP1_DPAD_DOWN) == CheckState.unChecked) {
                    buttons.put(GP1_DPAD_DOWN, CheckState.checked);
                }
            }

            if (gamepad1.dpad_left) {
                if (buttons.get(GP1_DPAD_LEFT) == CheckState.unChecked) {
                    buttons.put(GP1_DPAD_LEFT, CheckState.checked);
                }
            }

            if (gamepad1.dpad_right) {
                if (buttons.get(GP1_DPAD_RIGHT) == CheckState.unChecked) {
                    buttons.put(GP1_DPAD_RIGHT, CheckState.checked);
                }
            }

            if (gamepad1.left_bumper) {
                if (buttons.get(GP1_LEFT_BUMPER) == CheckState.unChecked) {
                    buttons.put(GP1_LEFT_BUMPER, CheckState.checked);
                }
            }

            if (gamepad1.right_bumper) {
                if (buttons.get(GP1_RIGHT_BUMPER) == CheckState.unChecked) {
                    buttons.put(GP1_RIGHT_BUMPER, CheckState.checked);
                }
            }

            if (gamepad1.left_stick_x != 0) {
                if (sticksAndTrigs.get(GP1_LEFT_STICK_X) == CheckState.unChecked) {
                    sticksAndTrigs.put(GP1_LEFT_STICK_X, CheckState.checked);
                }
            }

            if (gamepad1.left_stick_y != 0) {
                if (sticksAndTrigs.get(GP1_LEFT_STICK_Y) == CheckState.unChecked) {
                    sticksAndTrigs.put(GP1_LEFT_STICK_Y, CheckState.checked);
                }
            }

            if (gamepad1.right_stick_x != 0) {
                if (sticksAndTrigs.get(GP1_RIGHT_STICK_X) == CheckState.unChecked) {
                    sticksAndTrigs.put(GP1_RIGHT_STICK_X, CheckState.checked);
                }
            }

            //gamepad2
            if (gamepad2.a) {
                if (buttons.get(GP2_A) == CheckState.unChecked) {
                    buttons.put(GP2_A, CheckState.checked);
                }
            }

            if (gamepad2.b) {
                if (buttons.get(GP2_B) == CheckState.unChecked) {
                    buttons.put(GP2_B, CheckState.checked);
                }
            }

            if (gamepad2.start) {
                if (buttons.get(GP2_START) == CheckState.unChecked) {
                    buttons.put(GP2_START, CheckState.checked);
                }
            }

            if (gamepad2.guide) {
                if (buttons.get(GP2_GUIDE) == CheckState.unChecked) {
                    buttons.put(GP2_GUIDE, CheckState.checked);
                }
            }

            if (gamepad2.dpad_left) {
                if (buttons.get(GP2_DPAD_LEFT) == CheckState.unChecked) {
                    buttons.put(GP2_DPAD_LEFT, CheckState.checked);
                }
            }

            if (gamepad2.dpad_right) {
                if (buttons.get(GP2_DPAD_RIGHT) == CheckState.unChecked) {
                    buttons.put(GP2_DPAD_RIGHT, CheckState.checked);
                }
            }

            if (gamepad2.left_trigger != 0) {
                if (sticksAndTrigs.get(GP2_LEFT_TRIGGER) == CheckState.unChecked) {
                    sticksAndTrigs.put(GP2_LEFT_TRIGGER, CheckState.checked);
                }
            }

            if (gamepad2.right_trigger != 0) {
                if (sticksAndTrigs.get(GP2_RIGHT_TRIGGER) == CheckState.unChecked) {
                    sticksAndTrigs.put(GP2_RIGHT_TRIGGER, CheckState.checked);
                }
            }

            if (gamepad2.right_stick_y != 0) {
                if (sticksAndTrigs.get(GP2_RIGHT_STICK_Y) == CheckState.unChecked) {
                    sticksAndTrigs.put(GP2_RIGHT_STICK_Y, CheckState.checked);
                }
            }

            Iterator buttonIt = buttons.entrySet().iterator();
            Iterator stickIt = sticksAndTrigs.entrySet().iterator();

            while (buttonIt.hasNext()) {
                Map.Entry buttonPair = (Map.Entry) buttonIt.next();
                //telemetry.addData("The following buttons, dpad, and bumpers are not checked yet:", " ");
                if (buttonPair.getValue() == CheckState.unChecked) {
                    telemetry.addData((String) buttonPair.getKey(), "; ");
                }
            }

            while (stickIt.hasNext()) {
                Map.Entry stickPair = (Map.Entry) stickIt.next();
                //telemetry.addData("The following sticks and triggers are not checked yet:", " ");
                if (stickPair.getValue() == CheckState.unChecked) {
                    telemetry.addData((String) stickPair.getKey(), "; ");
                }

            }
        }
    }
}