package org.firstinspires.ftc.teamcode.Ramsete;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SixWheelDrive;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;


public class PathEngine{
	public String TAG = "PathEngineDebug";
	Path pathToDraw;
	String pathFileName;
	public double maxAccel;
	public double maxAngularAccel;
	public double[] prevIntercept;
	ArrayList<ArrayList<Point>> arcs;
	TimeBasedTaskList taskList;
	org.firstinspires.ftc.teamcode.hardware.Hardware hardware;
	LinearOpMode parentOP;
	public PathEngine(double maxAccel, double maxAngularAccel, String pathFileName, Hardware hardware, LinearOpMode parentOP){
		this.maxAccel = maxAccel;
		this.maxAngularAccel =maxAngularAccel;
		this.pathFileName = pathFileName;
		taskList = new TimeBasedTaskList();
		this.hardware = hardware;
		prevIntercept = new double[2];
		this.parentOP = parentOP;
	}
	public void init() {
		RobotLog.dd(TAG,"debugger works");
		pathToDraw = new Path(pathFileName);
		arcs = new ArrayList<ArrayList<Point>>();
		Double[] angles = new Double[pathToDraw.path.size()-1];
		for(int i = 0; i< angles.length; i++) {
			angles[i] = Math.atan2(pathToDraw.path.get(i+1).Y - pathToDraw.path.get(i).Y, pathToDraw.path.get(i+1).X - pathToDraw.path.get(i).X);
		}
		prevIntercept[0] = pathToDraw.path.get(0).X;
		prevIntercept[1] = pathToDraw.path.get(0).Y;
		for(int i = 0; i < pathToDraw.path.size()-2;i++) {
			if(Math.abs(MathFunctions.angleFormatting(angles[i]-angles[i+1])) > Math.toRadians(0.1)) {
				simulateArc arc;
				if(pathToDraw.path.get(i+1).angularAccel==null) {
					arc = new simulateArc(pathToDraw.path.get(i).X, pathToDraw.path.get(i).Y, pathToDraw.path.get(i+1).powerInitial, pathToDraw.path.get(i+1).powerFinal, MathFunctions.angleFormatting(angles[i+1]-angles[i]),angles[i],maxAngularAccel);
				}
				else {
					arc = new simulateArc(pathToDraw.path.get(i).X, pathToDraw.path.get(i).Y, pathToDraw.path.get(i+1).powerInitial, pathToDraw.path.get(i+1).powerFinal, MathFunctions.angleFormatting(angles[i+1]-angles[i]),angles[i],pathToDraw.path.get(i+1).angularAccel);
				}
				arc.calculateArc();
				double[] intercept = new double[2];
				double arcEndPointX = arc.arc.get(arc.arc.size()-1).X;
				double arcEndPointY = arc.arc.get(arc.arc.size()-1).Y;
				if(pathToDraw.path.get(i+1).X != pathToDraw.path.get(i).X && pathToDraw.path.get(i+1).X != pathToDraw.path.get(i+2).X) {
					double slopeLine1 = 1.0*(pathToDraw.path.get(i+1).Y-pathToDraw.path.get(i).Y)/(pathToDraw.path.get(i+1).X - pathToDraw.path.get(i).X);
					double interceptLine1 = arcEndPointY - slopeLine1 * arcEndPointX;
					double slopeLine2 = 1.0*(pathToDraw.path.get(i+2).Y-pathToDraw.path.get(i+1).Y)/(pathToDraw.path.get(i+2).X - pathToDraw.path.get(i+1).X);
					double interceptLine2 = pathToDraw.path.get(i+1).Y - slopeLine2 * pathToDraw.path.get(i+1).X;
					intercept = MathFunctions.findIntercept(slopeLine1, interceptLine1, slopeLine2, interceptLine2);
				}
				else if(pathToDraw.path.get(i+1).X == pathToDraw.path.get(i).X){
					intercept = new double[2];
					intercept[0] = arcEndPointX;
					double slopeLine2 = (pathToDraw.path.get(i+2).Y-pathToDraw.path.get(i+1).Y)/(pathToDraw.path.get(i+2).X - pathToDraw.path.get(i+1).X);
					double interceptLine2 = pathToDraw.path.get(i+1).Y - slopeLine2 * pathToDraw.path.get(i+1).X;
					intercept[1] = slopeLine2 * intercept[0] + interceptLine2;
				}
				else if(pathToDraw.path.get(i+1).X == pathToDraw.path.get(i+2).X){
					intercept = new double[2];
					double slopeLine1 = (pathToDraw.path.get(i+1).Y-pathToDraw.path.get(i).Y)/(pathToDraw.path.get(i+1).X - pathToDraw.path.get(i).X);
					double interceptLine1 = arcEndPointY - slopeLine1 * arcEndPointX;
					intercept[0] = pathToDraw.path.get(i+1).X;
					intercept[1] = slopeLine1 * intercept[0] + interceptLine1;
				}
				double translateX = intercept[0] - arcEndPointX;
				double translateY = intercept[1] - arcEndPointY;
				for(Point p: arc.arc) {
					p.X += translateX;
					p.Y += translateY;
				}
				for(Point p: arc.arcHalf1) {
					p.X += translateX;
					p.Y += translateY;
				}
				for(Point p: arc.arcHalf2) {
					p.X += translateX;
					p.Y += translateY;
				}
				double lineEndX = arc.arc.get(0).X;
				double lineEndY = arc.arc.get(0).Y;
				double lineLength = Math.hypot(lineEndX - prevIntercept[0],lineEndY-prevIntercept[1]);
				double lineAccel = (Math.pow(pathToDraw.path.get(i+1).powerInitial,2) - Math.pow(pathToDraw.path.get(i).powerFinal,2))/(2*lineLength);
				if(pathToDraw.path.get(i+1).powerInitial > pathToDraw.path.get(i).powerFinal) {
					double timeExponentialGrowth = -SixWheelDrive.kA/ SixWheelDrive.kV* Math.log((hardware.batteryVoltage- SixWheelDrive.kV*pathToDraw.path.get(i+1).powerInitial)/(hardware.batteryVoltage- SixWheelDrive.kV*pathToDraw.path.get(i).powerFinal));
					double distanceExponentialGrowth = -SixWheelDrive.kA/ SixWheelDrive.kV * pathToDraw.path.get(i).powerFinal * Math.pow(Math.E, -timeExponentialGrowth * SixWheelDrive.kV/ SixWheelDrive.kA) + hardware.batteryVoltage/ SixWheelDrive.kV*timeExponentialGrowth + hardware.batteryVoltage* SixWheelDrive.kA/ Math.pow(SixWheelDrive.kV,2)* Math.pow(Math.E, -timeExponentialGrowth* SixWheelDrive.kV/ SixWheelDrive.kA) + SixWheelDrive.kA/ SixWheelDrive.kV*pathToDraw.path.get(i).powerFinal - hardware.batteryVoltage* SixWheelDrive.kA/ Math.pow(SixWheelDrive.kV, 2);
					double lengthRemaining = lineLength-distanceExponentialGrowth;
					taskList.addTask(new ExponentialVeloGrowthTask(timeExponentialGrowth,pathToDraw.path.get(i).powerFinal, 0, new Point(prevIntercept[0], prevIntercept[1]), angles[i]));
					taskList.addTask(new Task(lengthRemaining/pathToDraw.path.get(i+1).powerInitial,pathToDraw.path.get(i+1).powerInitial,0, new Point(prevIntercept[0] + distanceExponentialGrowth * Math.cos(angles[i]), prevIntercept[1] + distanceExponentialGrowth * Math.sin(angles[i])), angles[i] ));
				}
				else {
					taskList.addTask(new Task(Math.abs(2*lineLength/(pathToDraw.path.get(i).powerFinal + pathToDraw.path.get(i+1).powerInitial)), pathToDraw.path.get(i).powerFinal,lineAccel,new Point(prevIntercept[0],prevIntercept[1]),angles[i]));
				}
				double arcTime = arc.turnTime(arc.deltaHeading,arc.maxAngularAccel);
				double arcAccel = (pathToDraw.path.get(i+1).powerFinal - pathToDraw.path.get(i+1).powerInitial)/(2*arcTime);
				taskList.addTask(new ArcTask(arcTime, pathToDraw.path.get(i+1).powerInitial, arcAccel,arc.arcHalf1.get(0),  angles[i],0,arc.maxAngularAccel,arc.arcHalf1));
				taskList.addTask(new ArcTask(arcTime, pathToDraw.path.get(i+1).powerInitial + arcAccel * arcTime,arcAccel,arc.arcHalf2.get(0),arc.halfTimeAngle,arc.halfTimeAngularVelocity,-arc.maxAngularAccel,arc.arcHalf2 ));
				prevIntercept = intercept;
				arcs.add(i, arc.arc);
			}
			else {
				double lineEndX = pathToDraw.path.get(i+1).X;
				double lineEndY = pathToDraw.path.get(i+1).Y;
				double lineLength = Math.hypot(lineEndX - prevIntercept[0],lineEndY-prevIntercept[1]);
				double lineAccel = (Math.pow(pathToDraw.path.get(i+1).powerInitial,2) - Math.pow(pathToDraw.path.get(i).powerFinal,2))/(2*lineLength);
				if(pathToDraw.path.get(i+1).powerInitial > pathToDraw.path.get(i).powerFinal) {
					double timeExponentialGrowth = -SixWheelDrive.kA/ SixWheelDrive.kV* Math.log((hardware.batteryVoltage- SixWheelDrive.kV*pathToDraw.path.get(i+1).powerInitial)/(hardware.batteryVoltage- SixWheelDrive.kV*pathToDraw.path.get(i).powerFinal));
					double distanceExponentialGrowth = -SixWheelDrive.kA/ SixWheelDrive.kV * pathToDraw.path.get(i).powerFinal * Math.pow(Math.E, -timeExponentialGrowth * SixWheelDrive.kV/ SixWheelDrive.kA) + hardware.batteryVoltage/ SixWheelDrive.kV*timeExponentialGrowth + hardware.batteryVoltage* SixWheelDrive.kA/ Math.pow(SixWheelDrive.kV,2)* Math.pow(Math.E, -timeExponentialGrowth* SixWheelDrive.kV/ SixWheelDrive.kA) + SixWheelDrive.kA/ SixWheelDrive.kV*pathToDraw.path.get(i).powerFinal - hardware.batteryVoltage* SixWheelDrive.kA/ Math.pow(SixWheelDrive.kV, 2);
					double lengthRemaining = lineLength-distanceExponentialGrowth;
					taskList.addTask(new ExponentialVeloGrowthTask(timeExponentialGrowth,pathToDraw.path.get(i).powerFinal, 0, new Point(prevIntercept[0], prevIntercept[1]), angles[i]));
					taskList.addTask(new Task(lengthRemaining/pathToDraw.path.get(i+1).powerInitial,pathToDraw.path.get(i+1).powerInitial,0, new Point(prevIntercept[0] + distanceExponentialGrowth * Math.cos(angles[i]), prevIntercept[1] + distanceExponentialGrowth * Math.sin(angles[i])), angles[i] ));
				}
				else {
					taskList.addTask(new Task(Math.abs(2*lineLength/(pathToDraw.path.get(i).powerFinal + pathToDraw.path.get(i+1).powerInitial)), pathToDraw.path.get(i).powerFinal,lineAccel,new Point(prevIntercept[0],prevIntercept[1]),angles[i]));
				}
				prevIntercept[0] = lineEndX;
				prevIntercept[1] = lineEndY;
				arcs.add(null);
			}
		}
		double lineEndX = pathToDraw.path.get(pathToDraw.path.size()-1).X;
		double lineEndY = pathToDraw.path.get(pathToDraw.path.size()-1).Y;
		double lineLength = Math.hypot(lineEndX - prevIntercept[0],lineEndY-prevIntercept[1]);
		double lineAccel = (Math.pow(pathToDraw.path.get(pathToDraw.path.size()-1).powerInitial,2) - Math.pow(pathToDraw.path.get(pathToDraw.path.size()-2).powerFinal,2))/(2*lineLength);
		taskList.addTask(new Task(Math.abs(2*lineLength/(pathToDraw.path.get(pathToDraw.path.size()-2).powerFinal + pathToDraw.path.get(pathToDraw.path.size()-1).powerInitial)), pathToDraw.path.get(pathToDraw.path.size()-2).powerFinal,lineAccel,new Point(prevIntercept[0],prevIntercept[1]),angles[angles.length-1]));
		prevIntercept[0] = lineEndX;
		prevIntercept[1] = lineEndY;
		arcs.add(null);
		taskList.writeAllData();
	}
	public void run(ElapsedTime time, double constantB, double constantC, boolean forward) {
		if(forward){
			hardware.setForward();
		}
		else{
			hardware.setBackwards();
		}
		double startTime = time.milliseconds();
		FileWriter writerDesiredKinematics;
		FileWriter writer;
		try {
			writerDesiredKinematics = new FileWriter("//sdcard//FIRST//ramsetedesiredkinematics.txt");
			 writer = new FileWriter("//sdcard//FIRST//RamseteMotionData.txt");
		}
		catch(IOException e){
			return;
		}
		RobotLog.dd(TAG, "right before while loop on run()");
		while((time.milliseconds() - startTime)/1000 < taskList.getTotalTime()&&!parentOP.isStopRequested() ){
			double currentTime = (time.milliseconds() - startTime)/1000;
			MotionData currentMotionData = taskList.getMotionData(currentTime);
			currentMotionData.desiredVelocity = currentMotionData.desiredVelocity/39.3701;
			currentMotionData.desiredPosition.X = currentMotionData.desiredPosition.X/39.3701;
			currentMotionData.desiredPosition.Y = currentMotionData.desiredPosition.Y/39.3701;
			double xPosMeters;
			double yPosMeters;
			xPosMeters = hardware.getXMeters();
			yPosMeters = hardware.getYMeters();
			double currentHeading = hardware.angle;
			if(!forward){
				currentMotionData.desiredHeading = currentMotionData.desiredHeading+ Math.toRadians(180);
				currentMotionData.desiredVelocity = -currentMotionData.desiredVelocity;
				currentMotionData.accel = -currentMotionData.accel;
			}
			double xErrorLocal = (currentMotionData.desiredPosition.X - xPosMeters) * Math.cos(currentHeading) + (currentMotionData.desiredPosition.Y - yPosMeters) * Math.sin(currentHeading);
			double yErrorLocal = (currentMotionData.desiredPosition.X - xPosMeters) * -Math.sin(currentHeading) + (currentMotionData.desiredPosition.Y - yPosMeters) * Math.cos(currentHeading);
			double headingError = MathFunctions.keepAngleWithin180Degrees(currentMotionData.desiredHeading - currentHeading);
			double constantK = MathFunctions.getConstantK(currentMotionData.desiredVelocity, currentMotionData.desiredAngularVelocity,  constantB,  constantC);
			double angularVelocityCommand = MathFunctions.getAngularVelocityCommand(currentMotionData.desiredVelocity, currentMotionData.desiredAngularVelocity, headingError, yErrorLocal, constantB, constantK);
			double velocityCommand = MathFunctions.velocityCommand(currentMotionData.desiredVelocity, headingError, xErrorLocal, constantK)*39.3701;
			hardware.sixWheelDrive.setMotion(velocityCommand,currentMotionData.accel,angularVelocityCommand, currentMotionData.AngularAccel);
			hardware.updatePID = true;
			RobotLog.dd(TAG, "angularoffset1 :" + constantK * headingError +", angularOffset2: "+ constantB*currentMotionData.desiredVelocity*yErrorLocal);
			RobotLog.dd(TAG, "constantK: " + constantK);
			RobotLog.dd(TAG, "Ex: " + xErrorLocal + ", Ey: " + yErrorLocal);
			RobotLog.dd(TAG, "desired Heading: " + currentMotionData.desiredHeading + ", Heading: "+ hardware.angle);
			RobotLog.dd(TAG,"angularVeloCommand: "+ angularVelocityCommand + ", angularVelo: " + currentMotionData.desiredAngularVelocity);
			RobotLog.dd(TAG,"veloCommand: " + velocityCommand + ", velo: " +currentMotionData.desiredVelocity);
			try {
				writerDesiredKinematics.write("Time: "+currentTime+", Heading: " + currentMotionData.desiredHeading + ", ActualHeading: "+currentHeading + ", CurrentVelo: " + hardware.localYVelocity + ", VeloCommand: " + velocityCommand + ", DesiredVelo: "+currentMotionData.desiredVelocity*39.3701 + ", LateralVelo: " + hardware.localXToReportToRamseteDesiredKinematics/hardware.deltaTime+", AngularVeloCommand: "+angularVelocityCommand+"\n");
				writer.write("desired X: " + currentMotionData.desiredPosition.X * 39.3701 + ", desired Y: " + currentMotionData.desiredPosition.Y * 39.3701 + ", current X: " + hardware.getX()  + ", current Y: " + hardware.getY() +"\n");
			}
			catch(IOException e){
				return;
			}
		}
		hardware.updatePID = false;
		try {
			hardware.sixWheelDrive.left.writer.close();
			hardware.sixWheelDrive.right.writer.close();
			writer.close();
			writerDesiredKinematics.close();
		}
		catch(IOException e){
			return;
		}
		hardware.updatePID = false;
		hardware.sixWheelDrive.LB.setPower(0);
		hardware.sixWheelDrive.LF.setPower(0);
		hardware.sixWheelDrive.RB.setPower(0);
		hardware.sixWheelDrive.RF.setPower(0);
		RobotLog.dd(TAG,"existed run()");
	}
}
