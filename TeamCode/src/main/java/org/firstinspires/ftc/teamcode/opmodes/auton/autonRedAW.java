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
@Autonomous(name = "Red auton audience wall", group = "Autonomous")
public class autonRedAW extends LinearOpMode {
//    private turret turretSubsystem = new turret();
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
    private final Pose path1 = poseFactory.of(10.2057, 9.7057, 256);
    private final Pose path1Control1 = poseFactory.of(41.5224, 59.4542, 0);
    private final Pose point2 = poseFactory.of(56.1033, 122.7827, 270);
    private final Pose point2Control1 = poseFactory.of(28.7651, 102.2378, 0);
    private final Pose point3 = poseFactory.of(49.346, 129.7495, 100);
    private final Pose point3Control1 = poseFactory.of(66.9006, 137.3353, 0);
    private final Pose point4 = poseFactory.of(56.1423, 122.5448, 270);
    private final Pose point4Control1 = poseFactory.of(39.2427, 115.4113, 0);
    private final Pose point5 = poseFactory.of(12.729, 94.269, 270);
    private final Pose point5Control1 = poseFactory.of(52.73, 102.5819, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> outtakeMotor.setPower(1.0)),
                intakeOn(),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path1()),
                follow(follower, path2()),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path3()),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path4()),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                intakeOff(),
                instant(() -> outtakeMotor.setPower(0)),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
//        turretSubsystem.init(hardwareMap);
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
//            turret.AimSolution aim =
//                    turretSubsystem.calculateAim(
//                            robotX,
//                            robotY,
//                            robotHeadingDegrees,
//                            redTargetX,
//                            redTargetYBelowMidline,
//                            redTargetYAboveMidline
//                    );
//            turretSubsystem.update(aim.targetAngle);

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", robotHeadingDegrees);
//            telemetry.addData("Turret target angle", aim.targetAngle);
//            telemetry.addData("Turret current angle", turretSubsystem.getCurrentTurretDegrees());
//            telemetry.addData("Turret error", turretSubsystem.getErrorDegrees());
//            telemetry.addData("Distance to target", aim.targetDistance);

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
//        turretSubsystem.stop();
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).linear(path1, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).linear(point2, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).linear(point4, point5);
    }
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
