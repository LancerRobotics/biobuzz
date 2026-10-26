
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lancersTeleopController;
import org.firstinspires.ftc.teamcode.subsystems.robot;

@TeleOp(name = "Lancers TeleOp Red", group = "TeleOp")
public class lancersTeleOpRed extends OpMode {

    private lancersTeleopController controller;

    @Override
    public void init() {
        controller = new lancersTeleopController(
                84.0,//Target x axis CHANGE
                61.0,//Target y axis 1 (below mid line) CHANGE
                83.0,//Target y axis 2 (above mid line) CHANGE
                robot.odoX,//Starting x axis
                robot.odoY,//Starting y axis
                robot.odoH//Starting heading
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
