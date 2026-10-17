package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lancersTeleopController;

@TeleOp(name = "Lancers TeleOp Red", group = "Lancers")
public class lancersTeleOpRed extends lancersTeleopController {
    @Override
    protected double getAimTargetX() {
        return 58.0;
    }

    @Override
    protected double getAimTargetYBelowMidline() {
        return 56.0;
    }

    @Override
    protected double getAimTargetYAboveMidline() {
        return 88.0;
    }

    @Override
    protected double getStartingX() {
        return 52.73;
    }

    @Override
    protected double getStartingY() {
        return 102.5819;
    }

    @Override
    protected double getStartingHeading() {
        return 0.0;
    }
}
