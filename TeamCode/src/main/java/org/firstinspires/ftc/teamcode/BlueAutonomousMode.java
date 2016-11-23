package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Blue Autonomous
 */
@Autonomous(name="Autonomous Blue", group="nb")
public class BlueAutonomousMode extends AutonomousMode {
    @Override
    boolean isBlueAlliance() {
        return true;
    }
}
