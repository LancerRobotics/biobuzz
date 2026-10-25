
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;

public class outtake {
    private final DcMotor outtakeMotorTwo;
    private final DcMotorEx outtakeMotor;
    private final Servo outtakeServo;
    // Distance in inches.
    // Replace these dummy values with tested values.
    private final double[] distances = {24.0, 36.0, 48.0, 60.0, 72.0};
    private final double[] flywheelVelocities = {900.0, 1100.0, 1300.0, 1500.0, 1700.0};
    public outtake(HardwareMap hardwareMap) {
        outtakeMotorTwo = hardwareMap.get(DcMotor.class, lancersBotConfig.OUTTAKE_MOTOR_TWO);
        outtakeMotor = hardwareMap.get(DcMotorEx.class, lancersBotConfig.OUTTAKE_MOTOR);
        outtakeServo = hardwareMap.get(Servo.class, lancersBotConfig.OUTTAKE_SERVO);
        outtakeMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setVelocity(0);
        outtakeMotorTwo.setPower(0);
    }

    public void toggleOuttake(boolean outtakeOn) {
        outtakeMotorTwo.setPower(outtakeOn ? 1.0 : 0.0);
    }
    public void setServoPos(double position) {
        outtakeServo.setPosition(
                Math.max(0.0, Math.min(1.0, position))
        );
    }
    // Find the required flywheel velocity for a given distance.
    private double getFlywheelVelocity(double distance) {
        int lastIndex = distances.length - 1;

        if (distance <= distances[0]) {
            return flywheelVelocities[0];
        }

        if (distance >= distances[lastIndex]) {
            return flywheelVelocities[lastIndex];
        }

        // Interpolate between the two nearest distances.
        for (int i = 0; i < lastIndex; i++) {
            if (distance >= distances[i]
                    && distance <= distances[i + 1]) {

                double fraction =
                        (distance - distances[i])
                                / (distances[i + 1] - distances[i]);

                return flywheelVelocities[i]
                        + fraction * (
                        flywheelVelocities[i + 1]
                                - flywheelVelocities[i]
                );
            }
        }
        return 0.0;
    }
    public void shoot(double distance) {
        if (Double.isNaN(distance)
                || Double.isInfinite(distance)
                || distance < 0) {
            stopShooter();
            return;
        }
        double velocity = getFlywheelVelocity(distance);
        outtakeMotor.setVelocity(velocity);
    }
    public void stopShooter() {
        outtakeMotor.setVelocity(0);
    }
    //stop all
    public void stop() {
        stopShooter();
        outtakeMotorTwo.setPower(0);
    }
}
