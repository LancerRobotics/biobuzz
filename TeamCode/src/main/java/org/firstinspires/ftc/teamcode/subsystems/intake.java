package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
public class intake {
    private final DcMotor intakeMotor;
    private double intakePower = 0;
    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(
                DcMotor.class,
                lancersBotConfig.INTAKE_MOTOR
        );
        intakeMotor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );
    }
    public void setPower(double power) {
        intakePower = power;
        intakeMotor.setPower(power);
    }
    public double getPower() {
        return intakePower;
    }
    public void stop() {
        intakePower = 0;
        intakeMotor.setPower(0);
    }
}