package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.UKF;
import org.firstinspires.ftc.teamcode.subsystems.robot;

@Autonomous(name = "UKF Drive Test", group = "Autonomous")
public class ukfDriveTest extends LinearOpMode {
    private static final double IN_TO_M = 0.0254;

    @Override
    public void runOpMode() {
        robot bot = new robot(hardwareMap, telemetry, false, false, true);
        bot.update();
        // null = no per-cycle console log (too slow for the robot loop)
        UKF ukf = new UKF(bot.getRobotX() * IN_TO_M, bot.getRobotY() * IN_TO_M,
                Math.toRadians(bot.getHeading()), null);

        waitForStart();
        while (opModeIsActive()) {
            bot.update();
            ukf.ingest(new UKF.SensorReading(getRuntime(),
                    bot.getRobotX() * IN_TO_M,
                    bot.getRobotY() * IN_TO_M,
                    Math.toRadians(bot.getHeading())));

            // UKF outputs tank-style left/right powers; convert to your mecanum drive.
            UKF.MotorCommands cmd = ukf.getMotorCommands();
            double forward = (cmd.frontLeft + cmd.frontRight) / 2.0;
            double rotate = (cmd.frontLeft - cmd.frontRight) / 2.0;
            bot.drive.driveMecanum(forward, 0, rotate);

            UKF.Estimate est = ukf.getEstimate();
            telemetry.addData("UKF X (in)", est.x / IN_TO_M);
            telemetry.addData("UKF Y (in)", est.y / IN_TO_M);
            telemetry.addData("UKF heading (deg)", Math.toDegrees(est.theta));
            telemetry.update();
        }
        bot.drive.stop();
    }
}