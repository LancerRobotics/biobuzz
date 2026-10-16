package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
public class drive {
    public static final double DEAD_ZONE_LIMIT = 0.06;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    public drive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(
                DcMotorEx.class,
                lancersBotConfig.FRONT_LEFT_MOTOR
        );
        leftRear = hardwareMap.get(
                DcMotorEx.class,
                lancersBotConfig.REAR_LEFT_MOTOR
        );
        rightFront = hardwareMap.get(
                DcMotorEx.class,
                lancersBotConfig.FRONT_RIGHT_MOTOR
        );
        rightRear = hardwareMap.get(
                DcMotorEx.class,
                lancersBotConfig.REAR_RIGHT_MOTOR
        );
        leftFront.setDirection(
                DcMotorSimple.Direction.REVERSE
        );
        leftRear.setDirection(
                DcMotorSimple.Direction.REVERSE
        );
        rightFront.setDirection(
                DcMotorSimple.Direction.FORWARD
        );
        rightRear.setDirection(
                DcMotorSimple.Direction.FORWARD
        );
    }
    public void driveMecanum(double y,double x,double r) {
        double flPower = y + x + r;
        double frPower = y - x - r;
        double blPower = y - x + r;
        double brPower = y + x - r;
        double max = Math.max(1.0, Math.max(
                Math.abs(flPower),
                Math.max(Math.abs(frPower),
                        Math.max(Math.abs(blPower), Math.abs(brPower)))
        ));
        leftFront.setPower(flPower / max);
        rightFront.setPower(frPower / max);
        leftRear.setPower(blPower / max);
        rightRear.setPower(brPower / max);
    }
    public static double respectDeadZones(double input) {
        if(Math.abs(input) < DEAD_ZONE_LIMIT) {
            return 0;
        }
        return input;
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}