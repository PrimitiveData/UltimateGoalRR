package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Intake {
    Motor intakeMotor;
    RegServo intakeDropperGuard;
    ContRotServo intakeServoStarboard;
    ContRotServo getIntakeServoPort;
    double holdIntakeUp = 0.36;
    double releaseIntake = 0;
    public Intake(Motor intakeMotor, RegServo intakeDropperGuard, ContRotServo intakeServoStarboard, ContRotServo getIntakeServoPort){
        this.intakeMotor = intakeMotor;
        this.intakeMotor.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.intakeDropperGuard =intakeDropperGuard;
        this.intakeServoStarboard = intakeServoStarboard;
        this.getIntakeServoPort = getIntakeServoPort;
    }
    public void turnIntake(double power){
        intakeMotor.setPower(power);
        if(power != 0) {
            intakeServoStarboard.setPower(1);
            getIntakeServoPort.setPower(0);
        }
    }
    public void dropIntake(){
        intakeDropperGuard.setPosition(releaseIntake);
    }
}
