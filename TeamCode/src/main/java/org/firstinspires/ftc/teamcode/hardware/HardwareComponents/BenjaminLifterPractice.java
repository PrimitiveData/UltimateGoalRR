package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class BenjaminLifterPractice {
    Motor liftermotor;

    public BenjaminLifterPractice(Motor liftmotor)
    {
        liftermotor = liftmotor;
        liftermotor.motor.setDirection(DcMotor.Direction.FORWARD);
        liftermotor.motor.setPower(0);
    }

    public void moveLiftPower(double power){
        liftermotor.setPower(power);
    }

    public void moveLiftEncoder(int encoderPos){
        if(liftermotor.currentPosition > encoderPos) {
            while (liftermotor.currentPosition > encoderPos){
                liftermotor.setPower(-1);
            }
        }
        if(liftermotor.currentPosition < encoderPos) {
            while (liftermotor.currentPosition < encoderPos){
                liftermotor.setPower(1);
            }
        }
    }
}
