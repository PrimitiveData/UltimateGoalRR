package org.firstinspires.ftc.teamcode.Ramsete;

public class MotionData {
	public double desiredHeading;
	public double desiredAngularVelocity;
	public double desiredVelocity;
	public Point desiredPosition;
	public double AngularAccel;
	public double accel;
	public MotionData(double desiredHeading, double desiredAngularVelocity, Point desiredPosition, double desiredVelocity, double AngularAccel, double accel) {
		this.desiredHeading = desiredHeading;
		this.desiredAngularVelocity = desiredAngularVelocity;
		this.desiredPosition = desiredPosition;
		this.desiredVelocity = desiredVelocity;
		this.AngularAccel = AngularAccel;
		this.accel = accel;
	}
}
