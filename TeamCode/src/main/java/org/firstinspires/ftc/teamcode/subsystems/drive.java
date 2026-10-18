package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.lancersBotConfig;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class drive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    public drive() {
    }
    public drive(HardwareMap hardwareMap) {
        init(hardwareMap);
    }
    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.FRONT_LEFT_MOTOR);
        frontRightMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.FRONT_RIGHT_MOTOR);
        backLeftMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.REAR_LEFT_MOTOR);
        backRightMotor = hardwareMap.get(DcMotor.class, lancersBotConfig.REAR_RIGHT_MOTOR);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = hardwareMap.get(IMU.class, lancersBotConfig.IMU);
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
    }
    public void driveFieldRel(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(strafe,forward);
        theta = AngleUnit.normalizeRadians(theta-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);
        driveMecanum(newForward, newStrafe, rotate);
    }

    private void driveMecanum(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;
        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        frontLeftMotor.setPower(frontLeftPower / maxPower);
        frontRightMotor.setPower(frontRightPower / maxPower);
        backLeftMotor.setPower(backLeftPower / maxPower);
        backRightMotor.setPower(backRightPower / maxPower);
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
