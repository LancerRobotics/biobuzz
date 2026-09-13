package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
public class drive {
    public static final double DEAD_ZONE_LIMIT = 0.15;
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
    public void driveMecanum(
            double ly,
            double lx,
            double rx,
            double multiplier
    ) {
        ly *= multiplier;
        lx *= multiplier;
        rx *= multiplier;
        double denominator = Math.max(
                Math.abs(ly) +
                        Math.abs(lx) +
                        Math.abs(rx),
                1.0
        );
        double fl = (ly + lx + rx) / denominator;
        double bl = (ly - lx + rx) / denominator;
        double fr = (ly - lx - rx) / denominator;
        double br = (ly + lx - rx) / denominator;
        leftFront.setPower(fl);
        leftRear.setPower(bl);
        rightFront.setPower(fr);
        rightRear.setPower(br);
    }
    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    public static double respectDeadZones(double input) {
        if(Math.abs(input) < DEAD_ZONE_LIMIT) {
            return 0;
        }
        return input;
    }
}