package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class WobblerArm {
    RegServo wobblerArm;
    //wobbler arm positions
    public double armGrabWobblePos = 0.0;
    public double armPushWobblePos = 0.0;
    public double armRaiseWobble = 0.39;
    public double armStartingPos = 0.56;
    public double armRestingPos = 0.69;
    RegServo wobblerClaw;
    public double clawReleasePos = 0.05;
    public double clawGrip = 0.35;
    public double clawRestingPos = 0.275;
    public ArmState armState = ArmState.START;
    public WobblerArm(RegServo wobblerArm, RegServo wobblerClaw){
        this.wobblerArm = wobblerArm;
        this.wobblerClaw = wobblerClaw;
    }
    public void moveArmToGrabPos(){
        wobblerArm.setPosition(armGrabWobblePos);
    }
    public void gripWobble(){
        wobblerClaw.setPosition(clawGrip);
    }
    public void goToClawRestingPos(){
        wobblerClaw.setPosition(clawRestingPos);
    }
    public void raiseWobble(){
        wobblerArm.setPosition(armRaiseWobble);
    }
    public void releaseWobble(){
        wobblerClaw.setPosition(clawReleasePos);
    }
    public void goToWobbleStartingPos(){
        wobblerArm.setPosition(armStartingPos);
    }
    public void goToAutoWobblerDropPosition(){wobblerArm.setPosition(0);}
    public void goToWobblerDropPosition(){wobblerArm.setPosition((armRaiseWobble+armGrabWobblePos)/2);}
    public void goToArmRestingPos(){
        wobblerArm.setPosition(armRestingPos);
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
