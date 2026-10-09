package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
public class Constants {
    public static MecanumConfig drivetrainConfig =
            new MecanumConfig(c -> {
                c.frontLeftName.set(
                        lancersBotConfig.FRONT_LEFT_MOTOR);
                c.backLeftName.set(
                        lancersBotConfig.REAR_LEFT_MOTOR);
                c.frontRightName.set(
                        lancersBotConfig.FRONT_RIGHT_MOTOR);
                c.backRightName.set(
                        lancersBotConfig.REAR_RIGHT_MOTOR);
                c.frontLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(
                        DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(
                        DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(
                        DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            });
    public static PinpointConfig localizerConfig =
            new PinpointConfig(c -> {
                c.name.set(lancersBotConfig.PINPOINT);
                // PLACEHOLDERS update when build finishes
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);
                // Update when build finishes
                c.xPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
            });

    public static ForesightConfig foresightConfig =
            new ForesightConfig(c -> {
                // PLACEHOLDER tuning values
                c.forwardTranslational.set(
                        Controller.proportional(0.3));
                c.strafeTranslational.set(
                        Controller.proportional(0.3));

                c.coast.set(
                        Controller.proportionalFeedforward(0.01));
                c.brake.set(
                        Controller.proportionalFeedforward(0.01));

                c.headingFeedback.set(
                        Controller.proportional(1.0));

                c.headingBrakeCoefficients.set(
                        Vector2D.cartesian(0.05, 0.005));

                c.linearBrakeCoefficients.set(
                        Matrix.diag(0.1, 0.1));

                c.quadraticBrakeCoefficients.set(
                        Matrix.diag(0.001, 0.001));

                // PLACEHOLDERS must tune
                c.maxAchievableForwardVelocity.set(40.0);
                c.maxAchievableStrafeVelocity.set(30.0);
                c.naturalForwardDeceleration.set(40.0);
                c.naturalStrafeDeceleration.set(40.0);
            });

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
