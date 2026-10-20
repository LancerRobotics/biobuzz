
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lancersTeleopController;

@TeleOp(name = "Lancers TeleOp Red")
public class lancersTeleOpRed extends OpMode {

    private lancersTeleopController controller;

    @Override
    public void init() {
        controller = new lancersTeleopController(
                84.0,//Target x axis CHANGE
                61.0,//Target y axis 1 (below mid line) CHANGE
                83.0,//Target y axis 2 (above mid line) CHANGE
                52.73,//Starting x axis
                102.5819,//Starting y axis
                180.0//Starting heading
        );

        controller.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        controller.loop(gamepad1, gamepad2);
    }

    @Override
    public void stop() {
        if (controller != null) {
            controller.stop();
        }
    }
}
/*
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
}*/
