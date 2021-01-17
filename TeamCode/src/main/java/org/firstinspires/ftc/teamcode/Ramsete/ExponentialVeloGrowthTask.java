package org.firstinspires.ftc.teamcode.Ramsete;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SixWheelDrive;

public class ExponentialVeloGrowthTask extends Task {
    double batteryVoltage;
    boolean firstCall;
    public ExponentialVeloGrowthTask(double timeTaken, double velocity, double acceleration, Point startPos, double startHeading) {
        super(timeTaken, velocity, acceleration, startPos, startHeading);
        firstCall = true;
        batteryVoltage = Hardware.getInstance().batteryVoltage;
    }

    @Override
    public Point getDesiredPosition(double timeStamp){
        double distTravelled = -SixWheelDrive.kA/ SixWheelDrive.kV*velocity* Math.pow(Math.E,-timeStamp* SixWheelDrive.kV/ SixWheelDrive.kA) + batteryVoltage/ SixWheelDrive.kV*timeStamp + batteryVoltage* SixWheelDrive.kA/ Math.pow(SixWheelDrive.kV,2)* Math.pow(Math.E,-timeStamp * SixWheelDrive.kV/ SixWheelDrive.kA)+ SixWheelDrive.kA/ SixWheelDrive.kV*velocity-batteryVoltage* SixWheelDrive.kA/ Math.pow(SixWheelDrive.kV,2);
        return new Point(startPos.X + distTravelled * Math.cos(startHeading), startPos.Y + distTravelled * Math.sin(startHeading));
    }
    @Override
    public double getDesiredVelocity(double timeStamp) {
        double toReturn = velocity* Math.pow(Math.E,-timeStamp* SixWheelDrive.kV/ SixWheelDrive.kA) + batteryVoltage/ SixWheelDrive.kV*(1- Math.pow(Math.E,-timeStamp* SixWheelDrive.kV/ SixWheelDrive.kA));
        Hardware.telemetry.addLine("desired Velo exponential growth: "+toReturn);
        Hardware.telemetry.update();
        return toReturn;
    }
    @Override
    public double getAccel(double timeStamp){
        return -SixWheelDrive.kV/ SixWheelDrive.kA*velocity* Math.pow(Math.E,-timeStamp* SixWheelDrive.kV/ SixWheelDrive.kA) + batteryVoltage/ SixWheelDrive.kA* Math.pow(Math.E,-timeStamp* SixWheelDrive.kV/ SixWheelDrive.kA);
    }
    @Override
    public boolean isExponentialGrowthTask(){
        return true;
    }
}
