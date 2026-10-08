package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "sampleAuton2Kyle", group = "Autonomous")
public class sampleAuton2Kyle extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.3685, 23.4714, 90);
    private final Pose point2 = poseFactory.of(12.6797, 46.4141, 180);
    private final Pose point2Control1 = poseFactory.of(54.5247, 39.3646, 0);
    private final Pose point2Control2 = poseFactory.of(14.7826, 25.7292, 0);
    private final Pose point3 = poseFactory.of(57.099, 119.0099, 270);
    private final Pose point3Control1 = poseFactory.of(15.5703, 105.8021, 0);
    private final Pose point4 = poseFactory.of(47.1523, 128.5906, 90);
    private final Pose point4Control1 = poseFactory.of(93.0508, 123.0599, 0);
    private final Pose point5Control1 = poseFactory.of(19.993, 126.851, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
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
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2).linear(path1, point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).linear(point2, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).linear(point3, point4);
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point3).linear(point4, point3);
    }
}
