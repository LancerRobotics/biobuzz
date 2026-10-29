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

@Autonomous(name = "Red Auton p-2 Shoot-Garden-Shoot-Flower-Shoot-Parking", group = "Autonomous")
public class autonRedSGSFP2 extends LinearOpMode{
    private turret turretSubsystem = new turret();

    private static final double redTargetX = 58.0;
    private static final double redTargetYBelowMidline = 56.0;
    private static final double redTargetYAboveMidline = 88.0;

    private Follower follower;
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private DcMotor outtakeMotorTwo;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59, 11, 90);
    private final Pose point1 = poseFactory.of(9.7569, 9.079, 265);
    private final Pose point1Control1 = poseFactory.of(11.9132, 16.5205, 0);
    private final Pose point2 = poseFactory.of(58, 117, 270);
    private final Pose point2Control1 = poseFactory.of(24.3849, 118.1581, 0);
    private final Pose point3 = poseFactory.of(52.1924, 129.1177, 115);
    private final Pose point3Control1 = poseFactory.of(65, 125, 0);
    private final Pose point4 = poseFactory.of(58, 117, 270);
    private final Pose point4Control1 = poseFactory.of(65, 125, 0);
    private final Pose point5 = poseFactory.of(11.4991, 115.1521, 0);

    //paths
    public Path path1() {
        return curve(start, point1Control1, point1).linear(start, point1);
    }
    public Path path2() {
        return curve(point1, point2Control1, point2).linear(point1, point2);
    }
    public Path path3() {
        return curve(point2, point3Control1, point3).linear(point2, point3);
    }
    public Path path4() {
        return curve(point3, point4Control1, point4).linear(point3, point4);
    }
    public Path path5() {
        return line(point4, point5).linear(point4, point5);
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
                follow(follower, path2()),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path3()),
                intakeOn(),
                waitMs(2000),
                intakeOff(),
                follow(follower, path4()),
                outtakeOn(),
                waitMs(2000),
                outtakeOff(),
                follow(follower, path5())
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


        while (opModeIsActive()){
            follower.update();
            Scheduler.execute();
            // you could replace follower.pose().x() with these if wanted
            // double robotX = follower.pose().x();
            // double robotY = follower.pose().y();
            double robotHeadingDegrees = Math.toDegrees(follower.pose().heading());

            // turret aiming
            turret.AimSolution aim =
                    turretSubsystem.calculateAim(
                            follower.pose().x(), // replace if wanted
                            follower.pose().y(), // replace if wanted
                            robotHeadingDegrees,
                            redTargetX,
                            redTargetYBelowMidline,
                            redTargetYAboveMidline
                    );
            turretSubsystem.update(aim.targetAngle);

            telemetry.addData("x", follower.pose().x()); // replace if wanted
            telemetry.addData("y", follower.pose().y()); // replace if wanted
            telemetry.addData("heading", robotHeadingDegrees);
            telemetry.addData("Turret target angle", aim.targetAngle);
            telemetry.addData("Turret current angle", turretSubsystem.getCurrentTurretDegrees());
            telemetry.addData("Turret error", turretSubsystem.getErrorDegrees());
            telemetry.addData("Distance to target", aim.targetDistance);


            if(follower.currentPath() != null){
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

