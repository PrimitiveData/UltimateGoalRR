package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class RegServo {
    public boolean writeRequested;
    public double position;
    public Servo servo;
    public RegServo(Servo servo){
        this.servo = servo;
        writeRequested = false;
        position = 0;
    }
    public void setPosition(double position){
        this.position = position;
        writeRequested = true;
    }
}
