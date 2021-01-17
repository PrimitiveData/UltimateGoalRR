package org.firstinspires.ftc.teamcode.PurePursuit;

public class Pose {
    public double X;
    public double Y;
    public double heading;
    public double translationPower;
    public double rotationPower;
    public Pose(double X, double Y, double heading, double translationPower, double rotationPower){
        this.X = X;
        this.Y = Y;
        this.heading = heading;
        this.translationPower = translationPower;
        this.rotationPower = rotationPower;
    }
}
