package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Red Autonomous
 */
@Autonomous(name="Autonomous Red", group="nb")
public class RedAutonomousMode extends AutonomousMode {
    @Override
    boolean isBlueAlliance() {
        return false;
    }
}
