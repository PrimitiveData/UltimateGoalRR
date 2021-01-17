package org.firstinspires.ftc.teamcode.Ramsete;

import java.util.ArrayList;

public class simulateArc {
	public double beginVelocity;
	public double endVelocity;
	public double maxAngularAccel;//rad/s/s
	public double deltaHeading;//radians
	public ArrayList<Point> arc = new ArrayList<Point>();
	public ArrayList<Point> arcHalf1 = new ArrayList<>();
	public ArrayList<Point> arcHalf2 = new ArrayList<>();
	double prevTime;
	double currentRotationalVelocity = 0;
	double heading;
	double X;
	double Y;
	boolean calculateArcFirstIteration = true;
	double startTime;
	double halfTimeAngle;
	double halfTimeAngularVelocity;
	boolean halfTimePassed = false;
	public simulateArc(double X, double Y, double beginVelocity, double endVelocity, double deltaHeading, double beginHeading, double maxAngularAccel) {
		this.beginVelocity = beginVelocity;
		this.endVelocity = endVelocity;
		this.deltaHeading = deltaHeading;
		heading = beginHeading;
		this.maxAngularAccel = maxAngularAccel;
		this.X = X;
		this.Y = Y;
	}
	public double turnTime(double deltaHeading, double maxAngularAccel) {
		return Math.sqrt(Math.abs(deltaHeading/maxAngularAccel));
	}
	public void calculateArc() {
		while(prevTime - startTime < 2*turnTime(deltaHeading, maxAngularAccel)) {
		if(calculateArcFirstIteration) {
			if(deltaHeading < 0) {
				maxAngularAccel = -maxAngularAccel;
			}
			prevTime = 0;
			startTime = prevTime;
			calculateArcFirstIteration = false;
		}
		double accel = (endVelocity - beginVelocity)/(2*turnTime(deltaHeading, maxAngularAccel));
		double currentTime = prevTime + 0.005;
		//System.out.println("currentTime: "+currentTime+", startTime: "+startTime + ", turnTime: "+turnTime(deltaHeading,maxAngularAccel));
		double deltaTime = currentTime - prevTime;
		if(currentTime > turnTime(deltaHeading,maxAngularAccel)) {
			if(!halfTimePassed){
				halfTimeAngle = heading;
				halfTimeAngularVelocity = currentRotationalVelocity;
				halfTimePassed = true;
			}
			currentRotationalVelocity += deltaTime * -maxAngularAccel;
		}
		else {
			currentRotationalVelocity += deltaTime * maxAngularAccel;
		}
		heading += deltaTime * currentRotationalVelocity;/*
		double headingToEnter;
		if((heading+"").substring((heading+"").length()-3,(heading+"").length()-2).equalsIgnoreCase("E")){
			String headingString = heading+"";
			headingString = headingString.substring(0,headingString.length()-3);
			headingToEnter = Double.parseDouble(headingString);
		}
		else {
			headingToEnter = heading;
		}*/
		double prevX = X;
		double prevY = Y;
		X += ((currentTime-startTime)*accel + beginVelocity)*deltaTime* Math.cos(heading);
		Y += ((currentTime-startTime)*accel + beginVelocity)*deltaTime* Math.sin(heading);
		if(currentTime < turnTime(deltaHeading,maxAngularAccel)){
			arcHalf1.add(new Point(X,Y,currentTime-startTime));
		}
		else{
			arcHalf2.add(new Point(X,Y,currentTime-startTime-turnTime(deltaHeading,maxAngularAccel)));
		}
		arc.add(new Point(X,Y,currentTime-startTime));
		//System.out.println("heading: " + heading + ", change X: "+ (X-prevX)+", change Y: " + (Y-prevY) + ", cos(heading): " + Math.cos(heading));
		prevTime = currentTime;
		}
	}
}
