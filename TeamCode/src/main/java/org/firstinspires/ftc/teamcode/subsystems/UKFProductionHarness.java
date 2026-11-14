package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Production harness for {@link UKF}: the same filter class the simulation uses, fed by the real
 * robot's sensors and driving the real motors. Sensor reads and motor writes are stubs to be
 * filled in with the robot SDK (e.g. from an OpMode's init / loop).
 *
 * <p>Usage from an OpMode: construct once in init(), call {@link #step()} every loop(), and
 * {@link #stop()} when the OpMode ends.
 */
public final class UKFProductionHarness {

    private final UKF ukf;

    public UKFProductionHarness() {
        // TODO: inject hardware handles here (hardwareMap motors, odometry computer, IMU).
        double[] start = readOdometry();
        double heading = readHeading();
        // Pass null instead of System.out to silence the per-cycle filter log on the robot.
        ukf = new UKF(start[0], start[1], heading, System.out);
    }

    /** One control cycle: read sensors, run the filter, write motors. */
    public void step() {
        double t = readTimestampSeconds();
        double[] odom = readOdometry();
        double heading = readHeading();

        ukf.ingest(new UKF.SensorReading(t, odom[0], odom[1], heading));

        UKF.MotorCommands cmd = ukf.getMotorCommands();
        writeMotors(cmd.frontLeft, cmd.frontRight, cmd.rearLeft, cmd.rearRight);
    }

    /** Current filtered pose, e.g. for telemetry. */
    public UKF.Estimate getEstimate() {
        return ukf.getEstimate();
    }

    /** Stops all motors. */
    public void stop() {
        writeMotors(0, 0, 0, 0);
    }

    // ---------------------------------------------------------------------------------------
    // Hardware stubs.
    // ---------------------------------------------------------------------------------------

    private double readTimestampSeconds() {
        // TODO: use the SDK clock if preferred (e.g. ElapsedTime.seconds()).
        return System.nanoTime() * 1e-9;
    }

    /** Returns {X, Y} in meters in the field frame. */
    private double[] readOdometry() {
        // TODO: read X/Y from the odometry device (e.g. GoBildaPinpointDriver.getPosition()),
        //       converted to meters.
        return new double[]{0.0, 0.0};
    }

    /** Returns heading in radians, counter-clockwise positive. */
    private double readHeading() {
        // TODO: read heading from the IMU (e.g. imu.getRobotYawPitchRollAngles().getYaw(RADIANS)).
        return 0.0;
    }

    private void writeMotors(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        // TODO: frontLeftMotor.setPower(frontLeft); frontRightMotor.setPower(frontRight);
        //       rearLeftMotor.setPower(rearLeft);   rearRightMotor.setPower(rearRight);
    }
}
