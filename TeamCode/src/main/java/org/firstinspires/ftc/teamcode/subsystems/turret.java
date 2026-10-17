package org.firstinspires.ftc.teamcode.subsystems;

public class turret {
    private static final double FIELD_MIDLINE_Y = 72.0;

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
        double targetDx = targetX - robotX;
        double targetDy = targetY - robotY;
        double targetDistance = Math.hypot(targetDx, targetDy);
        double fieldBearing = Math.toDegrees(Math.atan2(targetDy, targetDx));
        double turretTargetAngle = normalizeDegrees(fieldBearing - robotHeading);

        return new AimSolution(targetX, targetY, targetDistance, turretTargetAngle);
    }

    private double normalizeDegrees(double angle) {
        return ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
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
