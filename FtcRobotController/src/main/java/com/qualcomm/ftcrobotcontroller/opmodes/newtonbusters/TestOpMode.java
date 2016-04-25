package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

//created by Athena Zheng on 4/2/16

public class TestOpMode extends OpMode {

    final double LOWERLIMIT = -0.5;
    final double UPPERLIMIT = 0.5;

    //the state 'checked' means 'not stuck' as well
    enum CheckState {unChecked, checked, stuck}
    CheckState checkstate;

    HashMap<String, CheckState> buttons = new HashMap<String, CheckState>();
    HashMap<String, CheckState> sticksAndTrigs = new HashMap<String, CheckState>();

    @Override
    public void init() {

        //gamepad1
        buttons.put("gamepad1, button a", gamepad1.a ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, button b", gamepad1.b ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, button x", gamepad1.x ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, button y", gamepad1.y ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, guide", gamepad1.guide ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, dpad up", gamepad1.dpad_up ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, dpad down", gamepad1.dpad_down ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, dpad left", gamepad1.dpad_left ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, dpad right", gamepad1.dpad_right ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, left bumper", gamepad1.left_bumper ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad1, right bumper", gamepad1.right_bumper ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put("gamepad1, left stick x", gamepad1.left_stick_x != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put("gamepad1, left stick y", gamepad1.left_stick_y != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put("gamepad1, right stick x", gamepad1.right_stick_x != 0 ? CheckState.stuck : CheckState.unChecked);

        //gamepad2
        buttons.put("gamepad2, button a", gamepad2.a ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad2, button b", gamepad2.y ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad2, start", gamepad2.start ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad2, guide", gamepad2.guide ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad2, dpad left", gamepad2.dpad_left ? CheckState.stuck : CheckState.unChecked);
        buttons.put("gamepad2, dpad right", gamepad2.dpad_right ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put("gamepad2, left trigger", gamepad2.left_trigger != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put("gamepad2, right trigger", gamepad2.right_trigger != 0 ? CheckState.stuck : CheckState.unChecked);
        sticksAndTrigs.put("gamepad2, right stick y", gamepad2.right_stick_y != 0 ? CheckState.stuck : CheckState.unChecked);

        Iterator buttonIt = buttons.entrySet().iterator();
        Iterator stickIt = sticksAndTrigs.entrySet().iterator();

        while (buttonIt.hasNext()) {
            Map.Entry buttonPair = (Map.Entry) buttonIt.next();
            telemetry.addData("The following are stuck:", " ");
            if (buttonPair.getValue() == CheckState.stuck) {
                telemetry.addData((String)buttonPair.getKey(), "; ");
            }
        }

        while (stickIt.hasNext()) {
            Map.Entry stickPair = (Map.Entry) stickIt.next();
            if (stickPair.getValue() == CheckState.stuck) {
                telemetry.addData((String)stickPair.getKey(), "; ");
            }
        }


    }

    @Override
    public void loop() {

        //gamepad1
        if (gamepad1.a) {
            if (buttons.get("gamepad1, button a") == CheckState.unChecked) {
                buttons.put("gamepad1, button a", CheckState.checked);
            }
        }

        if (gamepad1.b) {
            if (buttons.get("gamepad1, button b") == CheckState.unChecked) {
                buttons.put("gamepad1, button b", CheckState.checked);
            }
        }

        if (gamepad1.x) {
            if (buttons.get("gamepad1, button x") == CheckState.unChecked) {
                buttons.put("gamepad1, button x", CheckState.checked);
            }
        }

        if (gamepad1.y) {
            if (buttons.get("gamepad1, button y") == CheckState.unChecked) {
                buttons.put("gamepad1, button y", CheckState.checked);
            }
        }

        if (gamepad1.guide) {
            if (buttons.get("gamepad1, guide") == CheckState.unChecked) {
                buttons.put("gamepad1, guide", CheckState.checked);
            }
        }

        if (gamepad1.dpad_up) {
            if (buttons.get("gamepad1, dpad up") == CheckState.unChecked) {
                buttons.put("gamepad1, dpad up", CheckState.checked);
            }
        }

        if (gamepad1.dpad_down) {
            if (buttons.get("gamepad1, dpad down") == CheckState.unChecked) {
                buttons.put("gamepad1, dpad down", CheckState.checked);
            }
        }

        if (gamepad1.dpad_left) {
            if (buttons.get("gamepad1, dpad left") == CheckState.unChecked) {
                buttons.put("gamepad1, dpad left", CheckState.checked);
            }
        }

        if (gamepad1.dpad_right) {
            if (buttons.get("gamepad1, dpad right") == CheckState.unChecked) {
                buttons.put("gamepad1, dpad right", CheckState.checked);
            }
        }

        if (gamepad1.left_bumper) {
            if (buttons.get("gamepad1, left bumper") == CheckState.unChecked) {
                buttons.put("gamepad1, left bumper", CheckState.checked);
            }
        }

        if (gamepad1.right_bumper) {
            if (buttons.get("gamepad1, right bumper") == CheckState.unChecked) {
                buttons.put("gamepad1, right bumper", CheckState.checked);
            }
        }

        if (gamepad1.left_stick_x < UPPERLIMIT || gamepad1.left_stick_x > LOWERLIMIT) {
            if (sticksAndTrigs.get("gamepad1, left stick x") == CheckState.unChecked) {
                sticksAndTrigs.put("gamepad1, left stick x", CheckState.checked);
            }
        }

        if (gamepad1.left_stick_y < UPPERLIMIT || gamepad1.left_stick_y > LOWERLIMIT) {
            if (sticksAndTrigs.get("gamepad1, left stick y") == CheckState.unChecked) {
                sticksAndTrigs.put("gamepad1, left stick y", CheckState.checked);
            }
        }

        if (gamepad1.right_stick_x < UPPERLIMIT || gamepad1.right_stick_x > LOWERLIMIT) {
            if (sticksAndTrigs.get("gamepad1, right stick x") == CheckState.unChecked) {
                sticksAndTrigs.put("gamepad1, right stick x", CheckState.checked);
            }
        }

        //gamepad2
        if (gamepad2.a) {
            if (buttons.get("gamepad2, button a") == CheckState.unChecked) {
                buttons.put("gamepad2, button a", CheckState.checked);
            }
        }

        if (gamepad2.b) {
            if (buttons.get("gamepad2, button b") == CheckState.unChecked) {
                buttons.put("gamepad2, button b", CheckState.checked);
            }
        }

        if (gamepad2.start) {
            if (buttons.get("gamepad2, start") == CheckState.unChecked) {
                buttons.put("gamepad2, start", CheckState.checked);
            }
        }

        if (gamepad2.guide) {
            if (buttons.get("gamepad2, guide") == CheckState.unChecked) {
                buttons.put("gamepad2, guide", CheckState.checked);
            }
        }

        if (gamepad2.dpad_left) {
            if (buttons.get("gamepad2, dpad left") == CheckState.unChecked) {
                buttons.put("gamepad2, dpad left", CheckState.checked);
            }
        }

        if (gamepad2.dpad_right) {
            if (buttons.get("gamepad2, dpad right") == CheckState.unChecked) {
                buttons.put("gamepad2, dpad right", CheckState.checked);
            }
        }

        if (gamepad2.left_trigger < UPPERLIMIT || gamepad2.left_trigger > LOWERLIMIT) {
            if (sticksAndTrigs.get("gamepad2, left trigger") == CheckState.unChecked) {
                sticksAndTrigs.put("gamepad2, left trigger", CheckState.checked);
            }
        }

        if (gamepad2.right_trigger < UPPERLIMIT || gamepad2.right_trigger > LOWERLIMIT) {
            if (sticksAndTrigs.get("gamepad2, right trigger") == CheckState.unChecked) {
                sticksAndTrigs.put("gamepad2, right trigger", CheckState.checked);
            }
        }

        if (gamepad2.right_stick_y < UPPERLIMIT || gamepad2.right_stick_y > LOWERLIMIT) {
            if (sticksAndTrigs.get("gamepad2, right stick y") == CheckState.unChecked) {
                sticksAndTrigs.put("gamepad2, right stick y", CheckState.checked);
            }
        }

        Iterator buttonIt = buttons.entrySet().iterator();
        Iterator stickIt = sticksAndTrigs.entrySet().iterator();


        while (buttonIt.hasNext()) {
            Map.Entry buttonPair = (Map.Entry) buttonIt.next();
            telemetry.addData("The following have not been checked yet:", " ");
            if (buttonPair.getValue() == CheckState.unChecked) {
                telemetry.addData((String) buttonPair.getKey(), "; ");
            }
        }

        while (stickIt.hasNext()) {
            Map.Entry stickPair = (Map.Entry) stickIt.next();
            if (stickPair.getValue() == CheckState.unChecked) {
                telemetry.addData((String) stickPair.getKey(), "; ");
            }
        }


    }
}