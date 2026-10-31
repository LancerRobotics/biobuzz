package org.firstinspires.ftc.teamcode.opmodes.auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// import static com.pedropathing.ivy.commands.Commands.*;

@Autonomous(name = "Blue Auton p-2 Parking", group = "Autonomous")
public class autonBlueP2 extends LinearOpMode {


    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

    private Follower follower;

    private final Pose start = poseFactory.of(9, 70, 0);
    private final Pose point1 = poseFactory.of(9, 118, 0);

    //paths
    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }

    //sequence
    public Command autoRoutine(){
        return sequential(
                follow(follower, path1())
        );
    }

    //bot running
    @Override
    public void runOpMode(){
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()){
            follower.update();
            Scheduler.execute();
            // you could replace follower.pose().x() in telemetry.addData if wanted
            // double robotX = follower.pose().x();
            // double robotY = follower.pose().y();
            double robotHeadingDegrees = Math.toDegrees(follower.pose().heading());

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", robotHeadingDegrees);

            if(follower.currentPath() != null){
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

    }
}

