package org.firstinspires.ftc.teamcode.subsystems;

import java.io.PrintStream;
import java.util.Locale;

/**
 * Unscented Kalman filter for a four-wheeled (skid / differential steer) robot.
 *
 * <p>Standalone: no simulation, hardware or FTC SDK dependencies. Only java.* is used.
 *
 * <p>State x = [X, Y, theta] (meters, meters, radians, theta wrapped to (-pi, pi]).
 * Motion inputs are the commanded forward velocity v (m/s) and turn rate omega (rad/s),
 * propagated through the exact nonlinear unicycle equations; no Jacobians are used anywhere.
 * Each cycle takes two measurements: odometry (X, Y) and a heading reading (theta), each with
 * its own noise variance. After the update, motor powers for FL, FR, RL, RR are computed with
 * differential drive and proportional correction toward a target five meters forward of the
 * starting pose. The velocity / turn rate those powers actually produce is used as the motion
 * input for the next prediction step.
 *
 * <p>Public API: {@link #ingest(SensorReading)}, {@link #getEstimate()}, {@link #getMotorCommands()}.
 */
public class UKF {

    // ---------------------------------------------------------------------------------------
    // Unscented transform constants (standard scaled unscented transform).
    // ---------------------------------------------------------------------------------------
    /** Spread of sigma points around the mean (0 < alpha <= 1). */
    public static final double ALPHA = 0.5;
    /** Prior-distribution knowledge; 2 is optimal for Gaussians. */
    public static final double BETA = 2.0;
    /** Secondary scaling parameter; 0 is the usual choice (3 - n is the other common one). */
    public static final double KAPPA = 0.0;

    /** State dimension: X, Y, theta. */
    public static final int N = 3;
    /** Measurement dimension: odometry X, odometry Y, heading. */
    public static final int M = 3;
    /** Number of sigma points, 2n + 1. */
    public static final int NUM_SIGMA = 2 * N + 1;

    // ---------------------------------------------------------------------------------------
    // Process noise Q (variance growth per second; scaled by dt each prediction).
    // ---------------------------------------------------------------------------------------
    public static final double Q_X = 0.02 * 0.02;         // m^2 / s
    public static final double Q_Y = 0.02 * 0.02;         // m^2 / s
    public static final double Q_THETA = Math.toRadians(2.0) * Math.toRadians(2.0); // rad^2 / s

    // ---------------------------------------------------------------------------------------
    // Measurement noise R: one parameter for odometry XY, one for heading.
    // ---------------------------------------------------------------------------------------
    public static final double R_ODOM_XY = 0.05 * 0.05;   // m^2   (odometry X and Y)
    public static final double R_HEADING = Math.toRadians(3.0) * Math.toRadians(3.0); // rad^2

    // ---------------------------------------------------------------------------------------
    // Initial uncertainty.
    // ---------------------------------------------------------------------------------------
    public static final double P0_XY = 0.01 * 0.01;
    public static final double P0_THETA = Math.toRadians(1.0) * Math.toRadians(1.0);

    // ---------------------------------------------------------------------------------------
    // Drive / controller constants.
    // ---------------------------------------------------------------------------------------
    /** Distance of the target straight ahead of the starting pose (m). */
    public static final double TARGET_DISTANCE = 5.0;
    /** Left-to-right wheel spacing (m). */
    public static final double TRACK_WIDTH = 0.35;
    /** Wheel surface speed at motor power 1.0 (m/s). */
    public static final double MAX_WHEEL_SPEED = 1.5;
    /** Proportional gain on distance-to-target (1/s). */
    public static final double KP_LINEAR = 0.8;
    /** Proportional gain on bearing-to-target error (1/s). */
    public static final double KP_ANGULAR = 2.5;
    /** Forward speed cap before wheel normalization (m/s). */
    public static final double MAX_LINEAR_SPEED = 1.0;
    /** Distance at which the robot is considered arrived and stops (m). */
    public static final double ARRIVAL_TOLERANCE = 0.02;
    /** |omega| below this uses the straight-line limit of the motion model (rad/s). */
    private static final double OMEGA_EPS = 1e-6;

    // ---------------------------------------------------------------------------------------
    // Unscented transform weights (derived once from ALPHA, BETA, KAPPA).
    // ---------------------------------------------------------------------------------------
    private static final double LAMBDA = ALPHA * ALPHA * (N + KAPPA) - N;
    private static final double[] WM = new double[NUM_SIGMA]; // mean weights
    private static final double[] WC = new double[NUM_SIGMA]; // covariance weights

    static {
        // The center sigma point's mean weight is lambda / (n + lambda).
        WM[0] = LAMBDA / (N + LAMBDA);
        // The center covariance weight adds the (1 - alpha^2 + beta) higher-order correction.
        WC[0] = WM[0] + (1.0 - ALPHA * ALPHA + BETA);
        for (int i = 1; i < NUM_SIGMA; i++) {
            // Every off-center sigma point shares the equal weight 1 / (2 (n + lambda)).
            WM[i] = 1.0 / (2.0 * (N + LAMBDA));
            WC[i] = WM[i];
        }
    }

    // ---------------------------------------------------------------------------------------
    // Public data types.
    // ---------------------------------------------------------------------------------------

    /** One cycle's sensor data: odometry X/Y and a heading, all stamped with one time. */
    public static final class SensorReading {
        public final double timestampSec;
        public final double odomX;
        public final double odomY;
        public final double heading;

        public SensorReading(double timestampSec, double odomX, double odomY, double heading) {
            this.timestampSec = timestampSec;
            this.odomX = odomX;
            this.odomY = odomY;
            this.heading = heading;
        }
    }

    /** Filtered pose and its covariance. */
    public static final class Estimate {
        public final double x;
        public final double y;
        public final double theta;
        public final double[][] covariance;

        Estimate(double[] state, double[][] cov) {
            this.x = state[0];
            this.y = state[1];
            this.theta = state[2];
            this.covariance = copy(cov);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "X=%.4f m  Y=%.4f m  theta=%.4f rad (%.2f deg)",
                    x, y, theta, Math.toDegrees(theta));
        }
    }

    /** Motor powers in [-1, 1] for each wheel. */
    public static final class MotorCommands {
        public final double frontLeft;
        public final double frontRight;
        public final double rearLeft;
        public final double rearRight;

        MotorCommands(double fl, double fr, double rl, double rr) {
            this.frontLeft = fl;
            this.frontRight = fr;
            this.rearLeft = rl;
            this.rearRight = rr;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "FL=%+.4f  FR=%+.4f  RL=%+.4f  RR=%+.4f",
                    frontLeft, frontRight, rearLeft, rearRight);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Filter state.
    // ---------------------------------------------------------------------------------------
    private final double[] x = new double[N];
    private final double[][] P = new double[N][N];
    private final double targetX;
    private final double targetY;

    private double commandedV = 0.0;      // m/s, input for the next prediction
    private double commandedOmega = 0.0;  // rad/s, input for the next prediction
    private MotorCommands motorCommands = new MotorCommands(0, 0, 0, 0);

    private double lastTimestamp = Double.NaN;
    private long cycle = 0;
    private PrintStream log;

    /** Creates a filter at the given starting pose, logging to System.out. */
    public UKF(double startX, double startY, double startTheta) {
        this(startX, startY, startTheta, System.out);
    }

    /** Creates a filter at the given starting pose; pass null for {@code log} to silence output. */
    public UKF(double startX, double startY, double startTheta, PrintStream log) {
        x[0] = startX;
        x[1] = startY;
        x[2] = wrapAngle(startTheta);
        P[0][0] = P0_XY;
        P[1][1] = P0_XY;
        P[2][2] = P0_THETA;
        targetX = startX + TARGET_DISTANCE * Math.cos(startTheta);
        targetY = startY + TARGET_DISTANCE * Math.sin(startTheta);
        this.log = log;
    }

    /** Redirects per-cycle logging; null disables it. */
    public void setLog(PrintStream log) {
        this.log = log;
    }

    // ---------------------------------------------------------------------------------------
    // Public API.
    // ---------------------------------------------------------------------------------------

    /**
     * Runs one full predict / update / control cycle for a timestamped reading.
     * The prediction uses the velocity and turn rate commanded at the end of the previous cycle.
     */
    public void ingest(SensorReading reading) {
        double dt = Double.isNaN(lastTimestamp) ? 0.0 : reading.timestampSec - lastTimestamp;
        if (dt < 0) {
            throw new IllegalArgumentException("Timestamps must be non-decreasing (dt=" + dt + ")");
        }
        lastTimestamp = reading.timestampSec;
        cycle++;

        logf("%n==================== UKF cycle %d  t=%.4f s  dt=%.4f s ====================%n",
                cycle, reading.timestampSec, dt);
        logf("Inputs for prediction: v=%.4f m/s  omega=%.4f rad/s%n", commandedV, commandedOmega);

        // ---------------- Predict ----------------
        double[][] sigma = generateSigmaPoints(x, P);
        logMatrixRows("Sigma points BEFORE motion model (rows = points, cols = X Y theta)", sigma);

        double[][] sigmaPred = new double[NUM_SIGMA][];
        for (int i = 0; i < NUM_SIGMA; i++) {
            // Each sigma point is pushed through the exact nonlinear unicycle motion model.
            sigmaPred[i] = motionModel(sigma[i], commandedV, commandedOmega, dt);
        }
        logMatrixRows("Sigma points AFTER motion model", sigmaPred);

        // The predicted mean is the weighted average of the transformed sigma points.
        double[] xPred = weightedMean(sigmaPred, 2);
        // The predicted covariance is the weighted outer-product sum of sigma deviations plus Q*dt.
        double[][] PPred = weightedCovariance(sigmaPred, xPred, sigmaPred, xPred, 2, 2);
        PPred[0][0] += Q_X * dt;
        PPred[1][1] += Q_Y * dt;
        PPred[2][2] += Q_THETA * dt;
        symmetrize(PPred);

        logVector("Predicted mean", xPred);
        logMatrix("Predicted covariance", PPred);

        // ---------------- Update ----------------
        double[] z = {reading.odomX, reading.odomY, wrapAngle(reading.heading)};
        logf("Raw measurement 1 (odometry):  X=%.4f m  Y=%.4f m%n", reading.odomX, reading.odomY);
        logf("Raw measurement 2 (heading):   theta=%.4f rad (%.2f deg)%n",
                reading.heading, Math.toDegrees(reading.heading));

        // Sigma points are redrawn from the predicted distribution so Q is reflected in the spread.
        double[][] sigmaUpd = generateSigmaPoints(xPred, PPred);
        double[][] sigmaZ = new double[NUM_SIGMA][];
        for (int i = 0; i < NUM_SIGMA; i++) {
            // Each predicted sigma point is mapped into measurement space by the measurement model.
            sigmaZ[i] = measurementModel(sigmaUpd[i]);
        }

        // The predicted measurement is the weighted average of the measurement-space sigma points.
        double[] zPred = weightedMean(sigmaZ, 2);
        // The innovation covariance is the weighted spread of measurement sigma points plus R.
        double[][] S = weightedCovariance(sigmaZ, zPred, sigmaZ, zPred, 2, 2);
        S[0][0] += R_ODOM_XY;
        S[1][1] += R_ODOM_XY;
        S[2][2] += R_HEADING;
        symmetrize(S);
        // The cross-covariance pairs each state deviation with its measurement deviation.
        double[][] Pxz = weightedCovariance(sigmaUpd, xPred, sigmaZ, zPred, 2, 2);
        logMatrix("Cross-covariance Pxz", Pxz);

        double[][] K = multiply(Pxz, invertSpd(S));
        logMatrix("Kalman gain K", K);

        double[] innovation = subtract(z, zPred, 2);
        double[] Ky = multiply(K, innovation);
        for (int i = 0; i < N; i++) {
            x[i] = xPred[i] + Ky[i];
        }
        x[2] = wrapAngle(x[2]);

        double[][] KSKt = multiply(multiply(K, S), transpose(K));
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                P[i][j] = PPred[i][j] - KSKt[i][j];
            }
        }
        symmetrize(P);

        logVector("Updated state estimate", x);
        logMatrix("Updated covariance", P);

        // ---------------- Control ----------------
        computeMotorCommands();
        logf("Motor commands: %s%n", motorCommands);
        logf("Next-cycle inputs: v=%.4f m/s  omega=%.4f rad/s  (target %.3f, %.3f)%n",
                commandedV, commandedOmega, targetX, targetY);
    }

    /** Returns the current filtered pose and covariance. */
    public Estimate getEstimate() {
        return new Estimate(x, P);
    }

    /** Returns the most recently computed motor powers. */
    public MotorCommands getMotorCommands() {
        return motorCommands;
    }

    // ---------------------------------------------------------------------------------------
    // Models.
    // ---------------------------------------------------------------------------------------

    /** Exact nonlinear unicycle integration over dt with constant v and omega. */
    static double[] motionModel(double[] s, double v, double omega, double dt) {
        double theta = s[2];
        double nx, ny;
        if (Math.abs(omega) > OMEGA_EPS) {
            double thetaEnd = theta + omega * dt;
            nx = s[0] + (v / omega) * (Math.sin(thetaEnd) - Math.sin(theta));
            ny = s[1] - (v / omega) * (Math.cos(thetaEnd) - Math.cos(theta));
        } else {
            nx = s[0] + v * dt * Math.cos(theta);
            ny = s[1] + v * dt * Math.sin(theta);
        }
        return new double[]{nx, ny, wrapAngle(theta + omega * dt)};
    }

    /** Measurement model: odometry observes X and Y directly; the heading sensor observes theta. */
    static double[] measurementModel(double[] s) {
        return new double[]{s[0], s[1], wrapAngle(s[2])};
    }

    // ---------------------------------------------------------------------------------------
    // Control.
    // ---------------------------------------------------------------------------------------

    private void computeMotorCommands() {
        double dx = targetX - x[0];
        double dy = targetY - x[1];
        double distance = Math.hypot(dx, dy);

        double v;
        double omega;
        if (distance < ARRIVAL_TOLERANCE) {
            v = 0.0;
            omega = 0.0;
        } else {
            double bearingError = wrapAngle(Math.atan2(dy, dx) - x[2]);
            // Drive backward rather than spin around if the target ends up behind the robot.
            double direction = 1.0;
            if (Math.abs(bearingError) > Math.PI / 2) {
                direction = -1.0;
                bearingError = wrapAngle(bearingError + Math.PI);
            }
            v = direction * Math.min(KP_LINEAR * distance, MAX_LINEAR_SPEED) * Math.cos(bearingError);
            omega = KP_ANGULAR * bearingError;
        }

        // Differential drive: each side's surface speed is v -/+ omega * track / 2.
        double left = v - omega * TRACK_WIDTH / 2.0;
        double right = v + omega * TRACK_WIDTH / 2.0;

        // Scale both sides together so neither exceeds max speed, preserving curvature.
        double maxSide = Math.max(Math.abs(left), Math.abs(right));
        if (maxSide > MAX_WHEEL_SPEED) {
            left *= MAX_WHEEL_SPEED / maxSide;
            right *= MAX_WHEEL_SPEED / maxSide;
        }

        double leftPower = left / MAX_WHEEL_SPEED;
        double rightPower = right / MAX_WHEEL_SPEED;
        motorCommands = new MotorCommands(leftPower, rightPower, leftPower, rightPower);

        // The motion actually commanded (after saturation) drives the next prediction.
        commandedV = (left + right) / 2.0;
        commandedOmega = (right - left) / TRACK_WIDTH;
    }

    // ---------------------------------------------------------------------------------------
    // Unscented transform helpers.
    // ---------------------------------------------------------------------------------------

    /** Builds the 2n+1 scaled sigma points from a mean and covariance. */
    static double[][] generateSigmaPoints(double[] mean, double[][] cov) {
        double[][] scaled = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                // The covariance is scaled by (n + lambda) so the points land at the scaled spread.
                scaled[i][j] = (N + LAMBDA) * cov[i][j];
            }
        }
        // The Cholesky factor gives a matrix square root whose columns are the spread directions.
        double[][] L = cholesky(scaled);

        double[][] sigma = new double[NUM_SIGMA][N];
        // Sigma point 0 sits exactly on the mean.
        sigma[0] = mean.clone();
        for (int j = 0; j < N; j++) {
            for (int i = 0; i < N; i++) {
                // Sigma point 1+j is the mean shifted forward along Cholesky column j.
                sigma[1 + j][i] = mean[i] + L[i][j];
                // Sigma point 1+n+j is the mean shifted backward along Cholesky column j.
                sigma[1 + N + j][i] = mean[i] - L[i][j];
            }
            // The heading of both shifted points is wrapped back into (-pi, pi].
            sigma[1 + j][2] = wrapAngle(sigma[1 + j][2]);
            sigma[1 + N + j][2] = wrapAngle(sigma[1 + N + j][2]);
        }
        return sigma;
    }

    /** Weighted mean of sigma points; the element at angleIndex is averaged on the circle. */
    static double[] weightedMean(double[][] pts, int angleIndex) {
        int dim = pts[0].length;
        double[] mean = new double[dim];
        double sinSum = 0.0;
        double cosSum = 0.0;
        for (int k = 0; k < pts.length; k++) {
            for (int i = 0; i < dim; i++) {
                if (i == angleIndex) {
                    continue;
                }
                // Each linear component of sigma point k contributes its value times WM[k].
                mean[i] += WM[k] * pts[k][i];
            }
            // The angular component contributes a weighted unit vector to avoid wrap-around bias.
            sinSum += WM[k] * Math.sin(pts[k][angleIndex]);
            cosSum += WM[k] * Math.cos(pts[k][angleIndex]);
        }
        mean[angleIndex] = Math.atan2(sinSum, cosSum);
        return mean;
    }

    /** Weighted cross-covariance sum_k WC[k] (a_k - aMean)(b_k - bMean)^T with angle-aware residuals. */
    static double[][] weightedCovariance(double[][] a, double[] aMean, double[][] b, double[] bMean,
                                         int aAngleIndex, int bAngleIndex) {
        int da = aMean.length;
        int db = bMean.length;
        double[][] out = new double[da][db];
        for (int k = 0; k < a.length; k++) {
            // Sigma point k's deviation from its mean is taken with the angle residual wrapped.
            double[] ra = subtract(a[k], aMean, aAngleIndex);
            double[] rb = subtract(b[k], bMean, bAngleIndex);
            for (int i = 0; i < da; i++) {
                for (int j = 0; j < db; j++) {
                    // The outer product of the two deviations is accumulated with weight WC[k].
                    out[i][j] += WC[k] * ra[i] * rb[j];
                }
            }
        }
        return out;
    }

    // ---------------------------------------------------------------------------------------
    // Small dense linear algebra (no external libraries).
    // ---------------------------------------------------------------------------------------

    static double wrapAngle(double a) {
        double r = Math.IEEEremainder(a, 2.0 * Math.PI);
        return r <= -Math.PI ? r + 2.0 * Math.PI : r;
    }

    private static double[] subtract(double[] a, double[] b, int angleIndex) {
        double[] r = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            r[i] = a[i] - b[i];
        }
        if (angleIndex >= 0 && angleIndex < a.length) {
            r[angleIndex] = wrapAngle(r[angleIndex]);
        }
        return r;
    }

    /** Lower-triangular Cholesky factor L with A = L L^T; throws if A is not positive definite. */
    static double[][] cholesky(double[][] A) {
        int n = A.length;
        double[][] L = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = A[i][j];
                for (int k = 0; k < j; k++) {
                    sum -= L[i][k] * L[j][k];
                }
                if (i == j) {
                    if (sum <= 0.0) {
                        throw new IllegalStateException("Matrix not positive definite in Cholesky");
                    }
                    L[i][i] = Math.sqrt(sum);
                } else {
                    L[i][j] = sum / L[j][j];
                }
            }
        }
        return L;
    }

    /** Inverse of a symmetric positive-definite matrix via Cholesky. */
    static double[][] invertSpd(double[][] A) {
        int n = A.length;
        double[][] L = cholesky(A);
        double[][] inv = new double[n][n];
        for (int c = 0; c < n; c++) {
            // Forward substitution: L y = e_c.
            double[] y = new double[n];
            for (int i = 0; i < n; i++) {
                double s = (i == c) ? 1.0 : 0.0;
                for (int k = 0; k < i; k++) {
                    s -= L[i][k] * y[k];
                }
                y[i] = s / L[i][i];
            }
            // Back substitution: L^T x = y.
            for (int i = n - 1; i >= 0; i--) {
                double s = y[i];
                for (int k = i + 1; k < n; k++) {
                    s -= L[k][i] * inv[k][c];
                }
                inv[i][c] = s / L[i][i];
            }
        }
        return inv;
    }

    private static double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, m = B.length, c = B[0].length;
        double[][] out = new double[r][c];
        for (int i = 0; i < r; i++) {
            for (int k = 0; k < m; k++) {
                for (int j = 0; j < c; j++) {
                    out[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return out;
    }

    private static double[] multiply(double[][] A, double[] v) {
        double[] out = new double[A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < v.length; j++) {
                out[i] += A[i][j] * v[j];
            }
        }
        return out;
    }

    private static double[][] transpose(double[][] A) {
        double[][] t = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                t[j][i] = A[i][j];
            }
        }
        return t;
    }

    private static void symmetrize(double[][] A) {
        for (int i = 0; i < A.length; i++) {
            for (int j = i + 1; j < A.length; j++) {
                double avg = 0.5 * (A[i][j] + A[j][i]);
                A[i][j] = avg;
                A[j][i] = avg;
            }
        }
    }

    private static double[][] copy(double[][] A) {
        double[][] c = new double[A.length][];
        for (int i = 0; i < A.length; i++) {
            c[i] = A[i].clone();
        }
        return c;
    }

    // ---------------------------------------------------------------------------------------
    // Logging.
    // ---------------------------------------------------------------------------------------

    private void logf(String fmt, Object... args) {
        if (log != null) {
            log.printf(Locale.US, fmt, args);
        }
    }

    private void logVector(String label, double[] v) {
        if (log == null) {
            return;
        }
        StringBuilder sb = new StringBuilder(label).append(": [");
        for (int i = 0; i < v.length; i++) {
            sb.append(String.format(Locale.US, "%s%12.6f", i == 0 ? "" : ", ", v[i]));
        }
        log.println(sb.append(" ]"));
    }

    private void logMatrix(String label, double[][] A) {
        if (log == null) {
            return;
        }
        log.println(label + ":");
        for (double[] row : A) {
            StringBuilder sb = new StringBuilder("  [");
            for (int j = 0; j < row.length; j++) {
                sb.append(String.format(Locale.US, "%s%14.6e", j == 0 ? "" : ", ", row[j]));
            }
            log.println(sb.append(" ]"));
        }
    }

    private void logMatrixRows(String label, double[][] pts) {
        if (log == null) {
            return;
        }
        log.println(label + ":");
        for (int k = 0; k < pts.length; k++) {
            log.printf(Locale.US, "  chi[%d] = [%12.6f, %12.6f, %12.6f ]%n",
                    k, pts[k][0], pts[k][1], pts[k][2]);
        }
    }
}
