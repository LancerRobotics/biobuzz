package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
public class intake {
    private final DcMotor intakeMotor;
    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(
                DcMotor.class,
                lancersBotConfig.INTAKE_MOTOR
        );
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(0);
    }
    public void setIntakePower(double power) {
        intakeMotor.setPower(Math.max(-0.8, Math.min(0.8, power)));
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
}
