package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Intake {
    Motor intakeMotor;
    RegServo intakeDropperGuard;
    ContRotServo intakeServoStarboard;
    ContRotServo intakeServoPort;
    double holdIntakeUp = 0.21;
    double bumperInit = 0.1438;
    double bumperRaised = 0.44;
    double releaseIntake = 0.58;

    RegServo leftBetorThingy;
    RegServo rightBetorThingy;

    double leftBetorThingyIn;
    double leftBetorThingyOut;
    double rightBetorThingyIn = 0.57;
    double rightBetorThingyOut = 0.32;

    double rightBetorThingyResting = 0.15;
    double leftBetorThingyResting;

    public Intake(Motor intakeMotor, RegServo intakeDropperGuard, ContRotServo intakeServoStarboard, ContRotServo intakeServoPort){
        this.intakeMotor = intakeMotor;
        this.intakeMotor.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.intakeDropperGuard =intakeDropperGuard;
        this.intakeServoStarboard = intakeServoStarboard;
        this.intakeServoPort = intakeServoPort;
    }

    public Intake(Motor intakeMotor, RegServo intakeDropperGuard, RegServo rightBetorThingy, RegServo leftBetorThingy){
        this.intakeMotor = intakeMotor;
        this.intakeMotor.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.intakeDropperGuard =intakeDropperGuard;
        this.leftBetorThingy = leftBetorThingy;
        this.rightBetorThingy = rightBetorThingy;
    }

    public void turnIntake(double power){
        intakeMotor.setPower(power);
        if(power != 0) {
            intakeServoStarboard.setPower(1);
            intakeServoPort.setPower(-1);
        }
        else{
            intakeServoStarboard.setPower(0);
            intakeServoPort.setPower(0);
        }
    }
    public void dropIntake(){
        intakeDropperGuard.setPosition(releaseIntake);
    }
    public void raiseBumper(){
        intakeDropperGuard.setPosition(bumperRaised);
    }

    public void turnIntakeExperimentalBetorThingy(double power){
        intakeMotor.setPower(power);
    }

    public void leftBetorThingyIn(){
        leftBetorThingy.setPosition(leftBetorThingyIn);
    }
    public void rightBetorThingyIn(){
        rightBetorThingy.setPosition(rightBetorThingyIn);
    }
    public void leftBetorThingyOut(){
        leftBetorThingy.setPosition(leftBetorThingyOut);
    }
    public void rightBetorThingyOut(){
        rightBetorThingy.setPosition(rightBetorThingyOut);
    }
    public void betorThingiesResting(){
        rightBetorThingy.setPosition(rightBetorThingyResting);
        leftBetorThingy.setPosition(leftBetorThingyResting);
    }
}
