package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Local JVM tests for {@link UKF}, driven through {@link UKFSimulationHarness}.
 * Run from Android Studio with the green arrow next to the class or any test method.
 */
public class UKFSimulationTest {

    private static final int CYCLES = 200;
    private static final double DT = 0.05;
    private static final long[] SEEDS = {1L, 7L, 42L, 1234L, 98765L};

    /** Full logged run (same as the harness main); open the Run panel to read the per-cycle log. */
    @Test
    public void fullyLoggedRunReachesTarget() {
        UKFSimulationHarness.Result r = UKFSimulationHarness.run(CYCLES, DT, 42L, System.out);
        assertTrue("robot should end within 5 cm of target, was " + r.truthDistanceToTarget(),
                r.truthDistanceToTarget() < 0.05);
    }

    /** The filter should beat the raw sensors and the robot should arrive, across several seeds. */
    @Test
    public void filterBeatsRawSensorsAndReachesTargetAcrossSeeds() {
        for (long seed : SEEDS) {
            UKFSimulationHarness.Result r = UKFSimulationHarness.run(CYCLES, DT, seed, null);
            String tag = "seed " + seed + ": ";

            assertTrue(tag + "estimate position RMS " + r.rmsPosErrorEst
                            + " should be under half the raw odometry RMS " + r.rmsPosErrorMeas,
                    r.rmsPosErrorEst < 0.5 * r.rmsPosErrorMeas);
            assertTrue(tag + "estimate heading RMS " + r.rmsHeadingErrorEst
                            + " should be under half the raw heading RMS " + r.rmsHeadingErrorMeas,
                    r.rmsHeadingErrorEst < 0.5 * r.rmsHeadingErrorMeas);
            assertTrue(tag + "robot should end within 5 cm of target, was "
                    + r.truthDistanceToTarget(), r.truthDistanceToTarget() < 0.05);
            assertEquals(tag + "final estimate X", r.truthX, r.finalEstimate.x, 0.05);
            assertEquals(tag + "final estimate Y", r.truthY, r.finalEstimate.y, 0.05);
        }
    }

    /** The filter's covariance must stay symmetric with a positive diagonal. */
    @Test
    public void covarianceStaysSymmetricAndPositive() {
        UKFSimulationHarness.Result r = UKFSimulationHarness.run(CYCLES, DT, 42L, null);
        double[][] p = r.finalEstimate.covariance;
        for (int i = 0; i < 3; i++) {
            assertTrue("P[" + i + "][" + i + "] should be positive", p[i][i] > 0);
            for (int j = 0; j < 3; j++) {
                assertEquals("P should be symmetric at " + i + "," + j, p[i][j], p[j][i], 1e-12);
            }
        }
    }

    /** Motor powers must always be within [-1, 1] and left/right sides paired. */
    @Test
    public void motorCommandsStayInRangeAndPaired() {
        UKF ukf = new UKF(0, 0, 0, null);
        for (int k = 0; k < 50; k++) {
            ukf.ingest(new UKF.SensorReading(k * DT, 0.1 * k, 0.0, 0.0));
            UKF.MotorCommands c = ukf.getMotorCommands();
            for (double p : new double[]{c.frontLeft, c.frontRight, c.rearLeft, c.rearRight}) {
                assertTrue("power out of range: " + p, p >= -1.0 && p <= 1.0);
            }
            assertEquals(c.frontLeft, c.rearLeft, 0.0);
            assertEquals(c.frontRight, c.rearRight, 0.0);
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsTimestampsGoingBackwards() {
        UKF ukf = new UKF(0, 0, 0, null);
        ukf.ingest(new UKF.SensorReading(1.0, 0, 0, 0));
        ukf.ingest(new UKF.SensorReading(0.5, 0, 0, 0));
    }
}
