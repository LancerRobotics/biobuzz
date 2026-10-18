package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;

public class outtake {
    private final DcMotor outtakeMotorTwo;
    private double intakePower = 0;
    private Servo outtakeServo;
    private DcMotorEx turretMotor;


    public outtake(HardwareMap hardwareMap) {
        outtakeMotorTwo = hardwareMap.get(
                DcMotor.class,
                lancersBotConfig.OUTTAKE_MOTOR_TWO
        );
        outtakeServo = hardwareMap.get(Servo.class, lancersBotConfig.OUTTAKE_SERVO);
        outtakeMotorTwo.setDirection(
                DcMotorSimple.Direction.REVERSE
        );
        turretMotor = hardwareMap.get(DcMotorEx.class, lancersBotConfig.OUTTAKE_ROTATION_MOTOR);

        // Preserve last year's motor direction convention.
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: the turret must be physically aligned to its chosen zero position before resetting the encoder.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(0.0);

    }
    public void toggleOuttake(boolean outtakeOn){
        outtakeMotorTwo.setPower(outtakeOn ? 1.0 : 0.0);
    }
    public void setServoPos(double angle){
        outtakeServo.setPosition(angle);
    }
    /*public void outtakeLuT(double distance){
        //if (distance >= )
    }*/
    public void stop() {
        outtakeMotorTwo.setPower(0);
    }
}

//TODO: add proj motion shi(needs tuning tho)
