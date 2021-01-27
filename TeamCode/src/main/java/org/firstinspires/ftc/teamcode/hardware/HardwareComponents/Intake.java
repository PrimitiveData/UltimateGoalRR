package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Intake {
    Motor intakeMotor;
    RegServo intakeDropperGuard;
    double holdIntakeUp = 0.36;
    double releaseIntake = 0;
    public Intake(Motor intakeMotor, RegServo intakeDropperGuard){
        this.intakeMotor = intakeMotor;
        this.intakeMotor.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.intakeDropperGuard =intakeDropperGuard;
    }
    public void turnIntake(double power){
        intakeMotor.setPower(power);
    }
    public void dropIntake(){
        intakeDropperGuard.setPosition(releaseIntake);
    }
}
