package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.pedropathing.api.Paths;
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
@Autonomous(name = "Red auton audience wall short", group = "Autonomous")
public class autonRedAWSHORT extends LinearOpMode {
    private turret turretSubsystem = new turret();
    //must verify constants
    private static final double redTargetX = 58.0;
    private static final double redTargetYBelowMidline = 56.0;
    private static final double redTargetYAboveMidline = 88.0;

    private Follower follower;
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private DcMotor outtakeMotorTwo;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(14.1342, 99.2856, 90);
    private final Pose path1Control1 = poseFactory.of(12.7248, 42.3509, 0);
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> outtakeMotor.setPower(1.0)),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        turretSubsystem.init(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.INTAKE_MOTOR);
        outtakeMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.OUTTAKE_MOTOR);
        outtakeMotorTwo = hardwareMap.get(DcMotor.class, lancersBotConfig.OUTTAKE_MOTOR_TWO);
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
        follower.setPose(start);
        follower.update();
        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            double robotX = follower.pose().x();
            double robotY = follower.pose().y();
            double robotHeadingDegrees = Math.toDegrees(follower.pose().heading());//chat said heading gets returned in radians but we doing posefollow degrees so will need to see this then adjust accordingly
            turret.AimSolution aim =
                    turretSubsystem.calculateAim(
                            robotX,
                            robotY,
                            robotHeadingDegrees,
                            redTargetX,
                            redTargetYBelowMidline,
                            redTargetYAboveMidline
                    );
            turretSubsystem.update(aim.targetAngle);

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", robotHeadingDegrees);
            telemetry.addData("Turret target angle", aim.targetAngle);
            telemetry.addData("Turret current angle", turretSubsystem.getCurrentTurretDegrees());
            telemetry.addData("Turret error", turretSubsystem.getErrorDegrees());
            telemetry.addData("Distance to target", aim.targetDistance);

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        turretSubsystem.stop();
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
    private Command intakeOn() {
        return instant(() -> intakeMotor.setPower(1.0));
    }

    private Command intakeOff() {
        return instant(() -> intakeMotor.setPower(0));
    }

    private Command outtakeOn() {
        return instant(() -> {
            outtakeMotorTwo.setPower(1.0);
        });
    }

    private Command outtakeOff() {
        return instant(() -> {
            outtakeMotorTwo.setPower(0);
        });
    }
}
