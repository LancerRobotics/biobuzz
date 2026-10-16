package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.subsystems.drive.respectDeadZones;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;

public class lancersTeleopController extends LinearOpMode {
    private static final double TRIGGER_PRESSED = 0.5;

    private boolean intakeForwardOn;
    private boolean intakeReverseOn;
    private boolean outtakeOn;
    private boolean outtakeTwoOn;

    private boolean previousIntake = false;
    private boolean previousOuttakeTwo = false;
    private boolean previousLeftTrigger;
    private boolean previousRightTrigger;

    @Override
    public void runOpMode() {
        drive driveSubsystem = new drive(hardwareMap);
        intake intakeSubsystem = new intake(hardwareMap);
        outtake outtakeSubsystem = new outtake(hardwareMap);

        DcMotor outtakeMotor = hardwareMap.get(
                DcMotor.class,
                lancersBotConfig.OUTTAKE_MOTOR
        );
        DcMotor outtakeMotorTwo = hardwareMap.get(
                DcMotor.class,
                lancersBotConfig.OUTTAKE_MOTOR_TWO
        );

        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
        intakeSubsystem.stop();
        driveSubsystem.stop();

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            driveSubsystem.driveMecanum(respectDeadZones(-gamepad1.left_stick_y), respectDeadZones(gamepad1.left_stick_x), respectDeadZones(gamepad1.right_stick_x));
            if (gamepad1.left_bumper && !previousIntake) {
                intakeForwardOn = !intakeForwardOn;
                intakeSubsystem.toggleIntake(intakeForwardOn);
            }
            previousIntake = gamepad1.left_bumper;
            if (gamepad1.right_bumper && !previousOuttakeTwo) {
                outtakeTwoOn = !outtakeTwoOn;
                outtakeSubsystem.toggleOuttake(outtakeTwoOn);
            }



            telemetry.addData("Intake forward", intakeForwardOn);
            telemetry.addData("Intake reverse", intakeReverseOn);
            telemetry.addData("Outtake", outtakeOn);
            telemetry.addData("Outtake two", outtakeTwoOn);
            telemetry.update();
        }

        driveSubsystem.stop();
        intakeSubsystem.stop();
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
    }


}
