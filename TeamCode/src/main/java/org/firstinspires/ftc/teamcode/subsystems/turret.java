
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;

public class turret {

    // Field geometry
    private static final double FIELD_MIDLINE_Y = 72.0;

    // Calibration from last year's robot.
    // These values must be verified on the current robot.
    private static final double TICKS_PER_DEGREE = -4.6733;
    private static final double TICKS_INTERCEPT = -3.6;

    // Cable-safe target interval: [-45, 315) degrees.
    private static final double WRAP_CENTER = 0.0;

    // Initial conservative control settings.
    private static final double MAX_POWER = 0.35;
    private static final double DEADBAND_DEGREES = 2.0;

    private DcMotorEx turretMotor;
    private double turretZeroOffset = 0.0;
    private double desiredTurretDegrees = 0.0;
    private double errorDegrees = 0.0;

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, lancersBotConfig.OUTTAKE_ROTATION_MOTOR);

        // Preserve last year's motor direction convention.
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: the turret must be physically aligned to its chosen zero position before resetting the encoder.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(0.0);
    }

    public AimSolution calculateAim(
            double robotX,
            double robotY,
            double robotHeading,
            double targetX,
            double targetYBelowMidline,
            double targetYAboveMidline
    ) {
        double targetY = robotY < FIELD_MIDLINE_Y
                ? targetYBelowMidline
                : targetYAboveMidline;

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double distance = Math.hypot(dx, dy);

        double fieldBearing =
                Math.toDegrees(Math.atan2(dy, dx));

        // Convert field-relative bearing to robot-relative angle.
        double relativeAngle =
                fieldBearing - robotHeading - turretZeroOffset;

        // Keep the same cable-wrap behavior as last yr.
        double turretAngle =
                wrapRelativeTo(relativeAngle, WRAP_CENTER);

        return new AimSolution(
                targetX,
                targetY,
                distance,
                turretAngle
        );
    }

    /*
    Maps angles into [center - 45, center + 315) aka what we had last yr.
    This preserves the old cable-wrap behavior.
     */
    private double wrapRelativeTo(double angle, double center) {
        double diff = angle - center;

        diff = ((diff + 45.0) % 360.0 + 360.0)
                % 360.0 - 45.0;

        return center + diff;
    }

    /*
     Move the turret toward the requested cable-wrapped angle.
     */
    public void update(double requestedAngle) {
        if (turretMotor == null) {
            return;
        }

        desiredTurretDegrees =
                wrapRelativeTo(requestedAngle, WRAP_CENTER);

        double targetTicks =
                TICKS_PER_DEGREE * desiredTurretDegrees
                        + TICKS_INTERCEPT;

        double currentTicks = -turretMotor.getCurrentPosition();

        double errorTicks = targetTicks - currentTicks;

        errorDegrees = errorTicks / -TICKS_PER_DEGREE;

        if (Math.abs(errorDegrees) <= DEADBAND_DEGREES) {
            turretMotor.setPower(0.0);
            return;
        }

        double absError = Math.abs(errorTicks);
        double powerMagnitude;

        // Coarse-to-fine power steps adapted from last year's code.
        if (absError >= 500) {
            powerMagnitude = MAX_POWER;
        } else if (absError >= 200) {
            powerMagnitude = MAX_POWER * 0.80;
        } else if (absError >= 100) {
            powerMagnitude = MAX_POWER * 0.55;
        } else if (absError >= 70) {
            powerMagnitude = MAX_POWER * 0.35;
        } else if (absError >= 30) {
            powerMagnitude = MAX_POWER * 0.23;
        } else {
            powerMagnitude = 0.0;
        }

        // Preserve last year's motor-power sign convention.
        if (errorTicks < 0) {
            turretMotor.setPower(powerMagnitude);
        } else {
            turretMotor.setPower(-powerMagnitude);
        }
    }

    public double getCurrentTurretDegrees() {
        if (turretMotor == null) {
            return 0.0;
        }

        double currentTicks = -turretMotor.getCurrentPosition();

        return (currentTicks - TICKS_INTERCEPT) / TICKS_PER_DEGREE;
    }

    public double getDesiredTurretDegrees() {
        return desiredTurretDegrees;
    }

    public double getErrorDegrees() {
        return errorDegrees;
    }

    /*
    Adjust the calibration offset in degrees.
    Positive values are subtracted from the calculated aim angle.
     */
    public void changeTurretOffset(double amount) {
        turretZeroOffset += amount;
    }

    public void stop() {
        if (turretMotor != null) {
            turretMotor.setPower(0.0);
        }
    }

    public static class AimSolution {
        public final double targetX;
        public final double targetY;
        public final double targetDistance;
        public final double targetAngle;

        private AimSolution(
                double targetX,
                double targetY,
                double targetDistance,
                double targetAngle
        ) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetDistance = targetDistance;
            this.targetAngle = targetAngle;
        }
    }
}
