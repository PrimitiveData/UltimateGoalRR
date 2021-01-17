package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Intake {
    Motor intakeMotor1;
    Motor intakeMotor2;
    RegServo intakeDropperGuard;
    double holdIntakeUp = 0.36;
    double releaseIntake = 0;
    public Intake(Motor intakeMotor1, Motor intakeMotor2, RegServo intakeDropperGuard){
        this.intakeMotor1 = intakeMotor1;
        this.intakeMotor2 = intakeMotor2;
        this.intakeMotor1.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.intakeMotor2.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.intakeDropperGuard =intakeDropperGuard;
    }
    public void turnIntake(double power){
        intakeMotor1.setPower(power);
        intakeMotor2.setPower(power);
    }
    public void dropIntake(){
        intakeDropperGuard.setPosition(releaseIntake);
    }
}
