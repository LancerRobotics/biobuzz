
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake;
import org.firstinspires.ftc.teamcode.subsystems.turret;

public class lancersTeleopController {

    // Robot subsystems
    private drive driveSubsystem;
    private intake intakeSubsystem;
    private outtake outtakeSubsystem;
    //private outtake servoSubsystem;
    private turret turretSubsystem;

    // Hardware
    private GoBildaPinpointDriver odometry;
    private DcMotor outtakeMotor;
    private DcMotor outtakeMotorTwo;

    private Telemetry telemetry;

    // Blue/Red configuration
    private final double aimTargetX;
    private final double aimTargetYBelowMidline;
    private final double aimTargetYAboveMidline;

    private final double startingX;
    private final double startingY;
    private final double startingHeading;

    // Mechanism states
    private boolean intakeForwardOn = false;
    private boolean intakeReverseOn = false;
    private boolean outtakeTwoOn = false;
    private boolean outtakeServoOn = false;

    // Button edge detection
    private boolean previousIntake = false;
    private boolean previousOuttakeTwo = false;
    private boolean previousOuttakeServo = false;

    private boolean initialized = false;

    public lancersTeleopController(
            double aimTargetX,
            double aimTargetYBelowMidline,
            double aimTargetYAboveMidline,
            double startingX,
            double startingY,
            double startingHeading
    ) {
        this.aimTargetX = aimTargetX;
        this.aimTargetYBelowMidline = aimTargetYBelowMidline;
        this.aimTargetYAboveMidline = aimTargetYAboveMidline;

        this.startingX = startingX;
        this.startingY = startingY;
        this.startingHeading = startingHeading;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize subsystems
        driveSubsystem = new drive();
        driveSubsystem.init(hardwareMap);
        intakeSubsystem = new intake(hardwareMap);
        outtakeSubsystem = new outtake(hardwareMap);
        //servoSubsystem = new outtake(hardwareMap);
        turretSubsystem = new turret();

        // Initialize motors
        outtakeMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.OUTTAKE_MOTOR);
        outtakeMotorTwo = hardwareMap.get(DcMotor.class, lancersBotConfig.OUTTAKE_MOTOR_TWO);
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);

        // Initialize odometry
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, lancersBotConfig.PINPOINT);
        odometry.setOffsets(5.5, -5.0, DistanceUnit.INCH);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();
        //adjust accordingly

        odometry.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        startingX,
                        startingY,
                        AngleUnit.DEGREES,
                        startingHeading
                )
        );

        driveSubsystem.stop();
        intakeSubsystem.stop();
        outtakeSubsystem.setServoPos(0);
        initialized = true;
        telemetry.addLine("Lancers TeleOp initialized");
        telemetry.update();
    }
    public void loop(Gamepad gamepad1, Gamepad gamepad2) {
        if (!initialized) {
            return;
        }

        //odometry
        odometry.update();
        double robotX = odometry.getPosX(DistanceUnit.INCH);
        double robotY = odometry.getPosY(DistanceUnit.INCH);
        double robotHeading = odometry.getHeading(AngleUnit.DEGREES);

        //Only job for gamepad 1: driving
        driveSubsystem.driveFieldRel(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        //Toggle intake - gamepad 2 left bumper
        if (gamepad2.left_bumper && !previousIntake) {
            intakeForwardOn = !intakeForwardOn;
            intakeSubsystem.toggleIntake(intakeForwardOn);
        }
        previousIntake = gamepad2.left_bumper;

        //Reverse intake when pressed - gamepad 2 x
        intakeReverseOn = gamepad2.x;
        intakeSubsystem.reverseIntake(intakeReverseOn);

        //Toggle feeder wheel - gamepad 2 right bumper
        if (gamepad2.right_bumper && !previousOuttakeTwo) {
            outtakeTwoOn = !outtakeTwoOn;
            outtakeSubsystem.toggleOuttake(outtakeTwoOn);
        }
        previousOuttakeTwo = gamepad2.right_bumper;

        //Toggle servo - gamepad 2 a
        if (gamepad2.a && !previousOuttakeServo) {
            outtakeServoOn = !outtakeServoOn;

            outtakeSubsystem.setServoPos(
                    outtakeServoOn ? 5 : 0
            );
        }
        previousOuttakeServo = gamepad2.a;

        //Turret
        turret.AimSolution aim =
                turretSubsystem.calculateAim(
                        robotX,
                        robotY,
                        robotHeading,
                        aimTargetX,
                        aimTargetYBelowMidline,
                        aimTargetYAboveMidline
                );
        turretSubsystem.update(aim.targetAngle);

        outtakeSubsystem.shoot(aim.targetDistance);
        if (gamepad2.right_trigger > 0.5) {
            outtakeSubsystem.shoot(aim.targetDistance);
        } else {
            outtakeSubsystem.stopShooter();
        }

        //Telemetry
        telemetry.addData("Intake forward", intakeForwardOn);
        telemetry.addData("Intake reverse", intakeReverseOn);
        telemetry.addData("Outtake/feeder", outtakeTwoOn);
        telemetry.addData("Servo position", outtakeServoOn);
        telemetry.addData("Odometry X (in)", robotX);
        telemetry.addData("Odometry Y (in)", robotY);
        telemetry.addData("Heading (deg)", robotHeading);
        telemetry.addData("Aim target X (in)", aim.targetX);
        telemetry.addData("Aim target Y (in)", aim.targetY);
        telemetry.addData("Aim distance (in)", aim.targetDistance);
        telemetry.addData("Turret target angle (deg)", aim.targetAngle);
        telemetry.update();
    }

    public void stop() {
        if (!initialized) {
            return;
        }
        driveSubsystem.stop();
        intakeSubsystem.stop();
        outtakeMotor.setPower(0);
        outtakeMotorTwo.setPower(0);
        initialized = false;
    }
}
