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
    double holdIntakeUp = 0.17;
    double bumperInit = 0.1438;
    double bumperRaised = 0.44;
    double releaseIntake = 0.55;

    RegServo leftBetorThingy;
    RegServo rightBetorThingy;

    double leftBetorThingyIn = 0.89693;
    double leftBetorThingyOut = 0.68224;
    double rightBetorThingyIn = 0.37327;
    double rightBetorThingyOut = 0.56787;

    double rightBetorThingyResting = rightBetorThingyIn;
    double leftBetorThingyResting = leftBetorThingyIn;
    double leftBetorThingyStarting = 0.7787;
    double rightBetorThingyStarting = 0.4235;


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
        if(power > 0) {
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
    public void holdIntakeUp() { intakeDropperGuard.setPosition(holdIntakeUp);}
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
    public void betorThingiesStarting(){
        leftBetorThingy.setPosition(leftBetorThingyStarting);
        rightBetorThingy.setPosition(rightBetorThingyStarting);
    }
}
