package org.firstinspires.ftc.teamcode.Ramsete;

public class Task {
	public double timeTaken;
	public double velocity;
	public double acceleration;
	public Point startPos;
	public double startHeading;
	public double angularAccel;
	public Task(double timeTaken, double velocity, double acceleration, Point startPos, double startHeading) {
		this.timeTaken = timeTaken;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.startPos = startPos;
		this.startHeading = startHeading;
		angularAccel = 0;
	}
	public Point getDesiredPosition(double timeStamp) {
		double distTravelled = velocity * timeStamp + 0.5 * acceleration * Math.pow(timeStamp, 2);
		return new Point(startPos.X + distTravelled * Math.cos(startHeading), startPos.Y + distTravelled * Math.sin(startHeading));
	}
	public double getDesiredVelocity(double timeStamp) {
		return velocity + acceleration*timeStamp;
	}
	public double getDesiredHeading(double timeStamp) {
		return startHeading;
	}
	public double getDesiredAngularVelocity(double timeStamp) {
		return 0;
	}
	public double getAccel(double timeStamp){
		return acceleration;
	}
	public boolean isArcTask(){
		return false;
	}
	public boolean isExponentialGrowthTask(){
		return false;
	}
}
