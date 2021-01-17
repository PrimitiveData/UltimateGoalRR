package org.firstinspires.ftc.teamcode.Ramsete;

import java.util.ArrayList;

public class ArcTask extends Task {
	double startAngularVelocity;
	ArrayList<Point> arc;
	public ArcTask(double timeTaken, double velocity, double acceleration, Point startPos, double startHeading, double startAngularVelocity, double angularAccel, ArrayList<Point> arc) {
		super(timeTaken, velocity, acceleration, startPos, startHeading);
		this.angularAccel = angularAccel;
		this.startAngularVelocity = startAngularVelocity;
		this.arc = arc;
	}
	@Override
	public Point getDesiredPosition(double timeStamp) {
		for(int i = 0; i < arc.size(); i++) {
			if(arc.get(i).timeStamp > timeStamp) {
				return new Point(arc.get(i).X-arc.get(0).X + startPos.X, arc.get(i).Y-arc.get(0).Y + startPos.Y);
			}
		}
		return new Point(arc.get(arc.size()-1).X-arc.get(0).X + startPos.X, arc.get(arc.size()-1).Y-arc.get(0).Y + startPos.Y);
	}
	@Override
	public double getDesiredHeading(double timeStamp) {
		return startHeading + startAngularVelocity * timeStamp + 0.5*angularAccel* Math.pow(timeStamp, 2);
	}
	@Override
	public double getDesiredAngularVelocity(double timeStamp) {
		return startAngularVelocity + timeStamp * angularAccel;
	}
	@Override
	public boolean isArcTask(){
		return true;
	}
}
