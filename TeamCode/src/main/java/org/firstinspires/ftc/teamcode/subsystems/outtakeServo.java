package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.lancersBotConfig;
public class outtakeServo {
    private Servo outtakeServo;
    public void init(HardwareMap hardwareMap){
        outtakeServo = hardwareMap.get(Servo.class, lancersBotConfig.OUTTAKE_SERVO);
    }
    public void setServoPos(double angle){
        outtakeServo.setPosition(angle);
    }

}
