package org.firstinspires.ftc.teamcode.opmodes.auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.turret;

@Autonomous(name = "Blue Auton Shoot-Flower-Parking", group = "Autonomous")
public class autonBlueSFP extends LinearOpMode{
//    private turret turretSubsystem = new turret();

    private static final double redTargetX = 58.0;
    private static final double redTargetYBelowMidline = 56.0;
    private static final double redTargetYAboveMidline = 88.0;

    private Follower follower;
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private DcMotor outtakeMotorTwo;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

    private final Pose start = poseFactory.of(59, 11, 270);
    private final Pose point1 = poseFactory.of(12.5902, 42.0679, 135);
    private final Pose point1Control1 = poseFactory.of(8.8191, 17.9723, 0);
    private final Pose point2Start = poseFactory.of(12.5902, 42.0679, 135);
    private final Pose point2 = poseFactory.of(13.1357, 102.323, 90);
    private final Pose point2Control1 = poseFactory.of(26.5945, 47.1778, 0);

    public Path path1() {
        return curve(start, point1Control1, point1).linear(start, point1);
    }
    public Path path2() {
        return curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Command autoRoutine(){
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


        while (opModeIsActive()){
            follower.update();
            Scheduler.execute();
            // you could replace follower.pose().x() with these if wanted
            // double robotX = follower.pose().x();
            // double robotY = follower.pose().y();
            double robotHeadingDegrees = Math.toDegrees(follower.pose().heading());

            // turret aiming
//            turret.AimSolution aim =
//                    turretSubsystem.calculateAim(
//                            follower.pose().x(), // replace if wanted
//                            follower.pose().y(), // replace if wanted
//                            robotHeadingDegrees,
//                            redTargetX,
//                            redTargetYBelowMidline,
//                            redTargetYAboveMidline
//                    );
//            turretSubsystem.update(aim.targetAngle);

            telemetry.addData("x", follower.pose().x()); // replace if wanted
            telemetry.addData("y", follower.pose().y()); // replace if wanted
            telemetry.addData("heading", robotHeadingDegrees);
//            telemetry.addData("Turret target angle", aim.targetAngle);
//            telemetry.addData("Turret current angle", turretSubsystem.getCurrentTurretDegrees());
//            telemetry.addData("Turret error", turretSubsystem.getErrorDegrees());
//            telemetry.addData("Distance to target", aim.targetDistance);


            if(follower.currentPath() != null){
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

