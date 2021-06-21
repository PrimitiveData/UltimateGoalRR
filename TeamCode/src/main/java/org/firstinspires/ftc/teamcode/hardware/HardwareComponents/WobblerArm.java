package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class WobblerArm {
    public RegServo wobblerArm1;
    public RegServo wobblerArm2;
    //wobbler arm positions
    public double wobblerArm2PositionWhenWobblerArm1IsZero=1 ;
    //ALL ARM POS VARIABLES ARE FOR WOBBLE ARM SERVO 1
    public double armGrabWobblePos=0.19;
    public double armRaiseWobble=0.57;
    public double armStartingPos=0.73;
    public double armRestingPos=0.92;
    public double armDropPos = 0.28;
    RegServo wobblerClaw;
    public double clawReleasePos = 0.607;
    public double clawGrip = 0.97;
    public double clawRestingPos = 0.91874;
    public ArmState armState = ArmState.START;
    public WobblerArm(RegServo wobblerArm1, RegServo wobblerArm2, RegServo wobblerClaw){
        this.wobblerArm2 = wobblerArm2;
        this.wobblerArm1 = wobblerArm1;
        this.wobblerClaw = wobblerClaw;
    }
    public void moveArmToGrabPos(){
        wobblerArm1.setPosition(armGrabWobblePos);
        wobblerArm2.setPosition(wobblerArm2PositionWhenWobblerArm1IsZero - armGrabWobblePos);
    }
    public void gripWobble(){
        wobblerClaw.setPosition(clawGrip);
    }
    public void goToClawRestingPos(){
        wobblerClaw.setPosition(clawRestingPos);
    }
    public void raiseWobble(){
        wobblerArm1.setPosition(armRaiseWobble);
        wobblerArm2.setPosition(wobblerArm2PositionWhenWobblerArm1IsZero - armRaiseWobble);
    }
    public void releaseWobble(){
        wobblerClaw.setPosition(clawReleasePos);
    }
    public void goToWobbleStartingPos(){
        wobblerArm1.setPosition(armStartingPos);
        wobblerArm2.setPosition(wobblerArm2PositionWhenWobblerArm1IsZero - armStartingPos);
    }
    public void goToWobblerDropPosition(){
        wobblerArm1.setPosition(armDropPos);
        wobblerArm2.setPosition(wobblerArm2PositionWhenWobblerArm1IsZero-armDropPos);
    }
    public void goToArmRestingPos(){
        wobblerArm1.setPosition(armRestingPos);
        wobblerArm2.setPosition(wobblerArm2PositionWhenWobblerArm1IsZero-armRestingPos);
    }
    public void goToWobblerDropPosition2(){
        wobblerArm1.setPosition((armDropPos+armGrabWobblePos)/2);
        wobblerArm2.setPosition(wobblerArm2PositionWhenWobblerArm1IsZero-(armDropPos+armGrabWobblePos/2));
    }
    public enum ArmState{
        START,
        GRIP,
        LIFT;
    }
    public void toggleArmState(){
        if(armState == ArmState.START){
            armState = ArmState.GRIP;
        }
        else if(armState == ArmState.GRIP){
            armState = ArmState.LIFT;
        }
        else{
            armState = ArmState.GRIP;
        }
        if(armState == ArmState.GRIP){
            moveArmToGrabPos();
        }
        else if(armState == ArmState.LIFT){
            raiseWobble();
        }else{
            goToArmRestingPos();
        }
    }
}
