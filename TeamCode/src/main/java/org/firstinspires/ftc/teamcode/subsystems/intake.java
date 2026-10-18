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
    public void toggleIntake(boolean intakeOn){
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
    }
    public void reverseIntake(boolean intakeReverse){intakeMotor.setPower(intakeReverse ? -1.0 : 0.0);};

    public void stop() {
        intakeMotor.setPower(0);
    }
}
