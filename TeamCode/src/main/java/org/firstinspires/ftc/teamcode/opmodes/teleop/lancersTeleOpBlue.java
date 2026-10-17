package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lancersTeleopController;

@TeleOp(name = "Lancers TeleOp Blue")
public class lancersTeleOpBlue extends lancersTeleopController {
    @Override
    protected double getAimTargetX() {
        return 84.0;
    }

    @Override
    protected double getAimTargetYBelowMidline() {
        return 61.0;
    }

    @Override
    protected double getAimTargetYAboveMidline() {
        return 83.0;
    }

    @Override
    protected double getStartingX() {
        return 144.0 - 52.73;
    }

    @Override
    protected double getStartingY() {
        return 144.0 - 102.5819;
    }

    @Override
    protected double getStartingHeading() {
        return 180.0;
    }
}
