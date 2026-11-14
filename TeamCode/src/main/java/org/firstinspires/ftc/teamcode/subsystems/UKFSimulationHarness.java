package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Locale;
import java.util.Random;

/**
 * Simulation harness for {@link UKF}: a ground-truth robot is driven by the filter's own motor
 * commands (with actuation noise), and noisy odometry / heading readings are synthesized from it.
 * Logs everything the filter prints plus truth, measurement, and error every cycle.
 *
 * <p>Run on a desktop JVM: {@code java ...UKFSimulationHarness [cycles] [dtSeconds] [seed]}.
 */
public final class UKFSimulationHarness {

    // Ground-truth noise (independent of the filter's Q and R, so the filter is not cheating).
    private static final double TRUE_ODOM_STD = 0.04;                    // m
    private static final double TRUE_HEADING_STD = Math.toRadians(2.5); // rad
    private static final double WHEEL_SLIP_STD = 0.03;                   // fraction of wheel speed
    private static final int TRUTH_SUBSTEPS = 10;

    private UKFSimulationHarness() {
    }

    public static void main(String[] args) {
        int cycles = args.length > 0 ? Integer.parseInt(args[0]) : 200;
        double dt = args.length > 1 ? Double.parseDouble(args[1]) : 0.05;
        long seed = args.length > 2 ? Long.parseLong(args[2]) : 42L;
        Random rng = new Random(seed);

        double[] truth = {0.0, 0.0, 0.0};
        UKF ukf = new UKF(truth[0], truth[1], truth[2], System.out);

        double t = 0.0;
        for (int c = 0; c < cycles; c++) {
            // Advance ground truth with the motor powers currently applied.
            if (c > 0) {
                UKF.MotorCommands cmd = ukf.getMotorCommands();
                double left = 0.5 * (cmd.frontLeft + cmd.rearLeft) * UKF.MAX_WHEEL_SPEED
                        * (1.0 + WHEEL_SLIP_STD * rng.nextGaussian());
                double right = 0.5 * (cmd.frontRight + cmd.rearRight) * UKF.MAX_WHEEL_SPEED
                        * (1.0 + WHEEL_SLIP_STD * rng.nextGaussian());
                double v = (left + right) / 2.0;
                double omega = (right - left) / UKF.TRACK_WIDTH;
                double h = dt / TRUTH_SUBSTEPS;
                for (int s = 0; s < TRUTH_SUBSTEPS; s++) {
                    truth[0] += v * h * Math.cos(truth[2]);
                    truth[1] += v * h * Math.sin(truth[2]);
                    truth[2] = UKF.wrapAngle(truth[2] + omega * h);
                }
                t += dt;
            }

            UKF.SensorReading reading = new UKF.SensorReading(
                    t,
                    truth[0] + TRUE_ODOM_STD * rng.nextGaussian(),
                    truth[1] + TRUE_ODOM_STD * rng.nextGaussian(),
                    UKF.wrapAngle(truth[2] + TRUE_HEADING_STD * rng.nextGaussian()));

            ukf.ingest(reading);

            UKF.Estimate est = ukf.getEstimate();
            System.out.printf(Locale.US, "[SIM] truth:    X=%.4f  Y=%.4f  theta=%.4f%n",
                    truth[0], truth[1], truth[2]);
            System.out.printf(Locale.US, "[SIM] measured: X=%.4f  Y=%.4f  theta=%.4f%n",
                    reading.odomX, reading.odomY, reading.heading);
            System.out.printf(Locale.US, "[SIM] estimate: %s%n", est);
            System.out.printf(Locale.US,
                    "[SIM] error:    pos=%.4f m (meas %.4f m)  heading=%.3f deg (meas %.3f deg)%n",
                    Math.hypot(est.x - truth[0], est.y - truth[1]),
                    Math.hypot(reading.odomX - truth[0], reading.odomY - truth[1]),
                    Math.toDegrees(UKF.wrapAngle(est.theta - truth[2])),
                    Math.toDegrees(UKF.wrapAngle(reading.heading - truth[2])));
        }

        UKF.Estimate fin = ukf.getEstimate();
        System.out.printf(Locale.US, "%n[SIM] DONE after %d cycles (%.2f s). Truth X=%.4f Y=%.4f; "
                        + "estimate X=%.4f Y=%.4f; distance to target (truth) %.4f m%n",
                cycles, t, truth[0], truth[1], fin.x, fin.y,
                Math.hypot(UKF.TARGET_DISTANCE - truth[0], truth[1]));
    }
}
