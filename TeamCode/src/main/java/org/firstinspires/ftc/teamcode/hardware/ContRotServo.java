package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;

public class ContRotServo {
    public boolean writeRequested;
    public double power;
    public CRServo servo;
    public ContRotServo(CRServo servo){
        this.servo = servo;
        writeRequested = false;
        power = 0;
    }
    public void setPower(double power){
        this.power = power;
        writeRequested = true;
    }
}
