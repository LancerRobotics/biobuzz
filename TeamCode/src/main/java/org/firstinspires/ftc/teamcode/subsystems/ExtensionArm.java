package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class ExtensionArm {

    // Setup/basic values
    private int MIN_TICKS = 0;
    private static int MAX_TICKS = 3000;
    private double NUDGE_POWER = -0.5;
    private double RUN_TO_POSITION_POWER = 0.75;

    private DcMotor liftMotor;
//    private TouchSensor liftMagneticLimitSwitch;

    private int targetLiftTicks = 0;
    private boolean wasAtMagneticLimit = false;

    public void init(HardwareMap hardwareMap){
        // Setting starting direction and freezing it in place
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Starting posistion
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Setting up Hall-Effect Sensor for Lift
//        liftMagneticLimitSwitch = hardwareMap.get(TouchSensor.class, "lift_Mag");

        targetLiftTicks = 0;
//        wasAtMagneticLimit = false;

    }

    // Commanding motor to move
    public void setTargetTicks(int ticks){
        targetLiftTicks = Range.clip(ticks, MIN_TICKS, MAX_TICKS);
        liftMotor.setTargetPosition(targetLiftTicks);

        if(liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        liftMotor.setPower(RUN_TO_POSITION_POWER);
    }

    public void setTargetPercentage(int percent){
        int ticks = (int) Range.scale(percent, 0, 100, MIN_TICKS, MAX_TICKS);
    }

//    public void updateData(){
//        boolean atLimit = liftMagneticLimitSwitch.isPressed();
//        if(atLimit && !wasAtMagneticLimit){
//            // have we returned to rest
//            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            liftMotor.setTargetPosition(0);
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        wasAtMagneticLimit = atLimit;
//    }


}
