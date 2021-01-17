package org.firstinspires.ftc.teamcode.Ramsete;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class TimeBasedTaskList {
	ArrayList<Task> taskList;
	public TimeBasedTaskList() {
		taskList = new ArrayList<Task>();

	}
	public void addTask(Task task) {
		taskList.add(task);
	}
	public double getTotalTime(){
		double totalTime = 0;
		for(Task t: taskList){
			totalTime += t.timeTaken;
		}
		return totalTime;
	}
	public MotionData getMotionData(double timeStamp) {
		double runningTime = 0;
		for(int i = 0; i < taskList.size(); i++) {
			runningTime+=taskList.get(i).timeTaken;
			if(runningTime > timeStamp) {
				runningTime-=taskList.get(i).timeTaken;
				MotionData toReturn;
				toReturn = new MotionData(taskList.get(i).getDesiredHeading(timeStamp-runningTime), taskList.get(i).getDesiredAngularVelocity(timeStamp-runningTime), taskList.get(i).getDesiredPosition(timeStamp - runningTime), taskList.get(i).getDesiredVelocity(timeStamp-runningTime), taskList.get(i).angularAccel,taskList.get(i).getAccel(timeStamp-runningTime));
				return toReturn;
			}
		}
		return new MotionData(0,0,new Point(0,0),0,0,0);
	}
	public void writeAllData() {
		FileWriter writer;
		try {
			 writer = new FileWriter("//sdcard//FIRST//RamseteKinematicData.txt");
		}
		catch(IOException e){
			return;
		}
		for(Task t: taskList){
			if(t.isArcTask()){
				ArcTask arct = (ArcTask)t;
				try {
					writer.write("timeTaken: "+ arct.timeTaken + ", Velo: " + arct.getDesiredVelocity(0) + ", AngularVelo: " + arct.getDesiredAngularVelocity(0) + ", Accel: " + arct.acceleration + ", AngularAccel: " + arct.angularAccel + "\n");
				}
				catch(IOException e){
					return;
				}
			}
			else{
				try {
					writer.write("timeTaken: "+t.timeTaken+", Velo: " + t.getDesiredVelocity(0) + ", AngularVelo: " + t.getDesiredAngularVelocity(0) + ", Accel: " + t.acceleration + ", AngularAccel: " + 0 + "\n");
				}
				catch(IOException e){
					return;
				}
			}
		}
		try {
			writer.close();
		}
		catch(IOException e){
			return;
		}
	}
}
