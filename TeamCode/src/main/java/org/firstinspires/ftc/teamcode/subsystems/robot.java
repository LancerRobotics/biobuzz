package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;


public class robot {
    public final drive drive;
    public final intake intake;
    public final outtake outtake;
    public final GoBildaPinpointDriver odo;
    private final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final boolean redMode;
    public final boolean autonMode;
    public robot(
            HardwareMap hardwareMap,
            Telemetry telem,
            boolean outtakeVelocity,
            boolean redMode,
            boolean autonMode
    ) {
        this.hardwareMap = hardwareMap;

        this.redMode = redMode;
        this.autonMode = autonMode;
        telemetry = telem;
        telemetry.setMsTransmissionInterval(50);
        drive = new drive(
                hardwareMap
        );
        intake = new intake(
                hardwareMap
        );
        outtake = new outtake(
                //TODO:hardwareMap,
                //TODO:OuttakeVelocity
        );
        odo = hardwareMap.get(
                GoBildaPinpointDriver.class,
                lancersBotConfig.PINPOINT
        );
        odo.setOffsets(
                5.5,
                -5,
                DistanceUnit.INCH
        );
        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );


        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );


        odo.resetPosAndIMU();

    }




    public void update() {

        odo.update();

    }





    public double getRobotX() {

        return odo.getPosX(
                DistanceUnit.INCH
        );

    }




    public double getRobotY() {

        return odo.getPosY(
                DistanceUnit.INCH
        );

    }




    public double getHeading() {

        return odo.getHeading(
                AngleUnit.DEGREES
        );

    }





    public void sendTelemetry() {


        telemetry.addLine("----------------");

        telemetry.addData(
                "X",
                "%.2f",
                getRobotX()
        );


        telemetry.addData(
                "Y",
                "%.2f",
                getRobotY()
        );


        telemetry.addData(
                "Heading",
                "%.2f",
                getHeading()
        );


        telemetry.addData(
                "Outtake Velocity",
                "%.0f"//,
                //TODO:outtake.getVelocity()
        );


        telemetry.addData(
                "Intake Power",
                intake.getPower()
        );


        telemetry.addLine("----------------");

    }





    public HardwareMap getHardwareMap() {

        return hardwareMap;

    }


}