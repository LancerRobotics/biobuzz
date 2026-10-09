package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.intake;

public class lancersTeleopController extends LinearOpMode {
    private static final double TRIGGER_PRESSED = 0.5;

    private boolean intakeForwardOn;
    private boolean intakeReverseOn;
    private boolean outtakeOn;
    private boolean outtakeTwoOn;

    private boolean previousLeftBumper;
    private boolean previousRightBumper;
    private boolean previousLeftTrigger;
    private boolean previousRightTrigger;

    @Override
    public void runOpMode() {
        drive driveSubsystem = new drive(hardwareMap);
        intake intakeSubsystem = new intake(hardwareMap);
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
            driveSubsystem.driveMecanum(
                    drive.respectDeadZones(-gamepad1.left_stick_y),
                    drive.respectDeadZones(gamepad1.left_stick_x),
                    drive.respectDeadZones(gamepad1.right_stick_x),
                    1.0
            );

            updateToggleInputs();

            intakeSubsystem.setPower(
                    intakeForwardOn ? 1.0 : (intakeReverseOn ? -1.0 : 0.0)
            );
            outtakeMotor.setPower(outtakeOn ? 1.0 : 0.0);
            outtakeMotorTwo.setPower(outtakeTwoOn ? 1.0 : 0.0);

            telemetry.addData("Intake forward", intakeForwardOn);
            telemetry.addData("Intake reverse", intakeReverseOn);
            telemetry.addData("Outtake", outtakeOn);
            telemetry.addData("Outtake two", outtakeTwoOn);
            telemetry.addLine("Gamepad 2 A: slides not configured");
            telemetry.update();
        }

        driveSubsystem.stop();
        intakeSubsystem.stop();
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
    }

    private void updateToggleInputs() {
        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;
        boolean leftTrigger = gamepad2.left_trigger > TRIGGER_PRESSED;
        boolean rightTrigger = gamepad2.right_trigger > TRIGGER_PRESSED;

        if (leftBumper && !previousLeftBumper) {
            intakeForwardOn = !intakeForwardOn;
            intakeReverseOn = false;
        }
        if (rightTrigger && !previousRightTrigger) {
            intakeReverseOn = !intakeReverseOn;
            intakeForwardOn = false;
        }
        if (leftTrigger && !previousLeftTrigger) {
            outtakeOn = !outtakeOn;
        }
        if (rightBumper && !previousRightBumper) {
            outtakeTwoOn = !outtakeTwoOn;
        }

        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;
        previousLeftTrigger = leftTrigger;
        previousRightTrigger = rightTrigger;
    }
}
