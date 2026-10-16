package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;

public class outtake {
    private final DcMotor outtakeMotorTwo;
    private double intakePower = 0;

    public outtake(HardwareMap hardwareMap) {
        outtakeMotorTwo = hardwareMap.get(
                DcMotor.class,
                lancersBotConfig.OUTTAKE_MOTOR_TWO
        );
        outtakeMotorTwo.setDirection(
                DcMotorSimple.Direction.REVERSE
        );
    }
    public void toggleOuttake(boolean outtakeOn){
        outtakeMotorTwo.setPower(outtakeOn ? 1.0 : 0.0);
    }
    public void stop() {
        outtakeMotorTwo.setPower(0);
    }
}

//TODO: add proj motion shi(needs tuning tho)
