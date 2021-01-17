package org.firstinspires.ftc.teamcode.Ramsete;

public class Point {
	public double X;
	public double Y;
	public double timeStamp; //milliseconds
	public Point(double X, double Y) {
		this.X = X;
		this.Y = Y;
	}
	public Point(double X, double Y, double timeStamp) {
		this.X = X;
		this.Y = Y;
		this.timeStamp = timeStamp;
	}
}
