
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lancersTeleopController;

@TeleOp(name = "Lancers TeleOp Blue")
public class lancersTeleOpBlue extends OpMode {

    private lancersTeleopController controller;

    @Override
    public void init() {
        controller = new lancersTeleopController(
                84.0,//Target x axis
                61.0,//Target y axis 1 (below mid line)
                83.0,//Target y axis 2 (above mid line)
                144.0 - 52.73,//Starting x axis
                144.0 - 102.5819,//Starting y axis
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
