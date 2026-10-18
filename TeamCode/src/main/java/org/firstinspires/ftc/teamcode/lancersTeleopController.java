package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.turret;

/* thx vincent and fire for this idea lol
Gamepad 1 controls:
Left stick drive, right stick rotate

Gamepad 2 controls:
Left bumper(toggle) intake, right bumper(toggle) outtake, x(hold) reverse intake
 */
public abstract class lancersTeleopController extends LinearOpMode {
    private boolean intakeForwardOn;
    private boolean intakeReverseOn;
    private boolean outtakeOn;
    private boolean outtakeTwoOn;
    private boolean outtakeServoOn;

    private boolean previousIntake = false;
    private boolean previousOuttakeTwo = false;
    private boolean previousOuttakeServo = false;
    private boolean previousLeftTrigger;
    private boolean previousRightTrigger;

    protected abstract double getAimTargetX();

    protected abstract double getAimTargetYBelowMidline();

    protected abstract double getAimTargetYAboveMidline();

    protected abstract double getStartingX();

    protected abstract double getStartingY();

    protected abstract double getStartingHeading();

    @Override
    public void runOpMode() {
        drive driveSubsystem = new drive();
        driveSubsystem.init(hardwareMap);
        intake intakeSubsystem = new intake(hardwareMap);
        outtake outtakeSubsystem = new outtake(hardwareMap);
        turret turretSubsystem = new turret();
        outtake outtakeServoSubsystem = new outtake(hardwareMap);
        GoBildaPinpointDriver odometry = hardwareMap.get(
                GoBildaPinpointDriver.class,
                lancersBotConfig.PINPOINT
        );
        odometry.setOffsets(5.5, -5, DistanceUnit.INCH);
        odometry.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odometry.resetPosAndIMU();
        sleep(250);
        odometry.setPosition(new Pose2D(
                DistanceUnit.INCH,
                getStartingX(),
                getStartingY(),
                AngleUnit.DEGREES,
                getStartingHeading()
        ));

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
        outtakeServoSubsystem.setServoPos(0);//temp must change when logan + build done

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();

            driveSubsystem.driveFieldRel(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (gamepad2.left_bumper && !previousIntake) {
                intakeForwardOn = !intakeForwardOn;
                intakeSubsystem.toggleIntake(intakeForwardOn);
            }
            previousIntake = gamepad1.left_bumper;
            if (gamepad2.x) {
                intakeSubsystem.reverseIntake(true);
            }
            if (gamepad2.right_bumper && !previousOuttakeTwo) {
                outtakeTwoOn = !outtakeTwoOn;
                outtakeSubsystem.toggleOuttake(outtakeTwoOn);
            }
            previousOuttakeTwo = gamepad1.right_bumper;
            if (gamepad2.a && !previousOuttakeServo) {
                outtakeServoOn = !outtakeServoOn;
                outtakeServoSubsystem.setServoPos(5);//temp must change when logan + build done
            }
            previousOuttakeTwo = gamepad1.right_bumper;



            double robotX = odometry.getPosX(DistanceUnit.INCH);
            double robotY = odometry.getPosY(DistanceUnit.INCH);
            double robotHeading = odometry.getHeading(AngleUnit.DEGREES);
            turret.AimSolution aim = turretSubsystem.calculateAim(
                    robotX,
                    robotY,
                    robotHeading,
                    getAimTargetX(),
                    getAimTargetYBelowMidline(),
                    getAimTargetYAboveMidline()
            );

            telemetry.addData("Intake forward", intakeForwardOn);
            telemetry.addData("Intake reverse", intakeReverseOn);
            telemetry.addData("Outtake", outtakeOn);
            telemetry.addData("Outtake two", outtakeTwoOn);
            telemetry.addData("Odometry X (in)", robotX);
            telemetry.addData("Odometry Y (in)", robotY);
            telemetry.addData("Odometry heading (deg)", robotHeading);
            telemetry.addData("Aim target X (in)", aim.targetX);
            telemetry.addData("Aim target Y (in)", aim.targetY);
            telemetry.addData("Aim target distance (in)", aim.targetDistance);
            telemetry.addData("Turret target angle (deg)", aim.targetAngle);
            telemetry.update();
        }

        driveSubsystem.stop();
        intakeSubsystem.stop();
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
    }

}
