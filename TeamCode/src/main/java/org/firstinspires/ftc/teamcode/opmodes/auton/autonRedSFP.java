package org.firstinspires.ftc.teamcode.opmodes.auton;
import static com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.api.PoseFactory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.turret;

@Autonomous(name = "Red Auton Shoot-Flower-Parking", group = "Autonomous")
public class autonRedSFP extends LinearOpMode{

    private turret turretSubsystem = new turret();

    private static final double redTargetX = 58.0;
    private static final double redTargetYBelowMidline = 56.0;
    private static final double redTargetYAboveMidline = 88.0;

    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private DcMotor outtakeMotorTwo;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 11, 270);
    private final Pose path1 = poseFactory.of(12.5902, 42.0679, 135);
    private final Pose point2Start = poseFactory.of(12.5902, 42.0679, 135);
    private final Pose point2 = poseFactory.of(13.1357, 102.323, 90);
    private final Pose point2Control1 = poseFactory.of(43.2631, 46.6283, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(()-> outtakeMotor.setPower(0.9)),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path1()),
                intakeOn(),
                waitMs(2000),
                intakeOff(),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }
        // commands
    private Command intakeOn() {
        return instant(() -> intakeMotor.setPower(1.0));
    }
    private Command intakeOff() {
        return instant(() -> intakeMotor.setPower(0));
    }
    private Command outtakeOn() {
        return instant(() -> {
            outtakeMotorTwo.setPower(0.9);
        });
    }
    private Command outtakeOff() {
        return instant(() -> {
            outtakeMotorTwo.setPower(0);
        });
    }
}

