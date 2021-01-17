package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BenjaminLifterPIDPractice extends GenericPID {
    public double kStatic;
    public BenjaminLifterPIDPractice(double kP, double kI, double kD, double kS, ElapsedTime time)
    {
        super(kP, kI, kD, time);
        kStatic = kS;
    }

    @Override
    public double updateCurrentStateAndGetOutput(double currentState) {
        return super.updateCurrentStateAndGetOutput(currentState) + kStatic;
    }
}
