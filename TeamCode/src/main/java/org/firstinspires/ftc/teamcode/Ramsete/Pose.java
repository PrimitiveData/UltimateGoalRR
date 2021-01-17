package org.firstinspires.ftc.teamcode.Ramsete;

public class Pose {
	public double X;
	public double Y;
	public double powerInitial;
	public double powerFinal;
	public Double angularAccel;
	public Pose(double X, double Y, double powerInitial, double powerFinal) {
		this.X = X;
		this.Y = Y;
		this.powerInitial = powerInitial;
		this.powerFinal = powerFinal;
		this.angularAccel = null;
	}
	public Pose(double X, double Y, double powerInitial, double powerFinal, Double angularAccel) {
		this.X = X;
		this.Y = Y;
		this.powerInitial = powerInitial;
		this.powerFinal = powerFinal;
		this.angularAccel = angularAccel;
	}

}
