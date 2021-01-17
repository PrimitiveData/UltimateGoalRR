package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

public class PPrunner {
    String fileNameForPoints;
    File fileContainingPoints;
    ArrayList<Pose> spline = new ArrayList<Pose>();
    HardwareMecanum hardware;
    LinearOpMode parentOP;
    FileWriter writer;

    public PPrunner(String fileNameForPoints, HardwareMecanum hardware, LinearOpMode parentOP){
        this.parentOP = parentOP;
        this.hardware = hardware;
        this.fileNameForPoints = fileNameForPoints;
        fileContainingPoints = new File(this.fileNameForPoints);
        Scanner scnr;
        try {
            scnr = new Scanner(fileContainingPoints);
        }
        catch(FileNotFoundException e){
            return;
        }
        while(scnr.hasNextLine()){
            String pointInStringFormat = scnr.nextLine();
            int xCoordPosition = pointInStringFormat.indexOf("X:");
            String xCoordString = pointInStringFormat.substring(xCoordPosition + 2);
            for(int i = 0; i < xCoordString.length(); i++){
                if(xCoordString.substring(i,i+1).equals(" ")){
                    xCoordString = xCoordString.substring(0,i);
                    i=xCoordString.length();
                }
            }
            double xCoord = Double.parseDouble(xCoordString);
            int yCoordPosition = pointInStringFormat.indexOf("Y:");
            String yCoordString = pointInStringFormat.substring(yCoordPosition + 2);
            for(int i = 0; i < yCoordString.length(); i++){
                if(yCoordString.substring(i,i+1).equals(" ")){
                    yCoordString = yCoordString.substring(0,i);
                    i=yCoordString.length();
                }
            }
            double yCoord = Double.parseDouble(yCoordString);
            int headingposition = pointInStringFormat.indexOf("Heading:");
            String headingString = pointInStringFormat.substring(headingposition + 8);
            for(int i = 0; i < headingString.length(); i++){
                if(headingString.substring(i,i+1).equals(" ")){
                    headingString = headingString.substring(0,i);
                    i=headingString.length();
                }
            }
            double heading = Double.parseDouble(headingString);
            int translationPowerPosition = pointInStringFormat.indexOf("TranslationPower:");
            String translationPowerString = pointInStringFormat.substring(translationPowerPosition + 17);
            for(int i = 0; i < translationPowerString.length(); i++){
                if(translationPowerString.substring(i,i+1).equals(" ")){
                    translationPowerString = translationPowerString.substring(0,i);
                    i=translationPowerString.length();
                }
            }
            double translationPower = Double.parseDouble(translationPowerString);
            int turningPowerPosition = pointInStringFormat.indexOf("TurnPower:");
            String turnPowerString = pointInStringFormat.substring(turningPowerPosition + 10);
            for(int i = 0; i < turnPowerString.length(); i++){
                if(turnPowerString.substring(i,i+1).equals(" ")){
                    turnPowerString = turnPowerString.substring(0,i);
                    i=turnPowerString.length();
                }
            }
            double turnPower = Double.parseDouble(turnPowerString);
            spline.add(new Pose(xCoord,yCoord, Math.toRadians(heading),translationPower,turnPower));
        }
        try {
            writer = new FileWriter("//sdcard//FIRST//RamseteMotionData.txt");
        }
        catch(
                IOException e){
            return;
        }
    }
    public int getSplinePoint(){
        double minDist = Math.hypot(spline.get(0).X, spline.get(0).Y);
        int minDistIndex = 0;
        int i;
        for(i = 1; i < spline.size();i++){
            double currentDist = Math.hypot(spline.get(i).X,spline.get(i).Y);
            if(currentDist < minDist){
                minDist = currentDist;
                minDistIndex = i;
            }
        }
        return minDistIndex;
    }
    public int getTargetPoint(int STEP_SIZE){
        int p = getSplinePoint();
        if(p < spline.size() - STEP_SIZE){
            return p + STEP_SIZE;
        }
        else{
            return spline.size()-1;
        }
    }
    public void start(double endMarginOfError, double lookAheadDist, double errorCorrection, Gamepad g){
        boolean splineFinished = false;
        Pose absoluteEndPoint = spline.get(spline.size() - 1);
        while(!splineFinished && !parentOP.isStopRequested() && !g.a){
            double[][] allData = new double[spline.size()-1][3];
            for(int i = 0; i < spline.size() - 1;i++){
                double[] lineToCurrentPointData = MathFunctions.lineToPointDistanceAndClosestPoint(spline.get(i).X, spline.get(i).Y,spline.get(i+1).X,spline.get(i+1).Y,hardware.getX(),hardware.getY());
                allData[i] = lineToCurrentPointData;
            }
            double minDist = allData[0][0];
            int minDistLine = 0;
            for(int p = 1; p < allData.length; p++){
                if(allData[p][0] < minDist){
                    minDist = allData[p][0];
                    minDistLine = p;
                }
            }
            double percentOfCurrentLineTravelled = MathFunctions.findPercentageOfCurrentLineTravelled(spline.get(minDistLine).X,spline.get(minDistLine).Y, spline.get(minDistLine+1).X,spline.get(minDistLine+1).Y,allData[minDistLine][1],allData[minDistLine][2]);
            double[] targetPoint = getTargetPoint(minDistLine, percentOfCurrentLineTravelled, lookAheadDist);
            double targetX = spline.get((int)targetPoint[0]).X + (spline.get((int)targetPoint[0] + 1).X - spline.get((int)targetPoint[0]).X) * targetPoint[1];
            double targetY = spline.get((int)targetPoint[0]).Y + (spline.get((int)targetPoint[0] + 1).Y - spline.get((int)targetPoint[0]).Y) * targetPoint[1];
            setPowersForTargetPoint(new Pose(allData[minDistLine][1], allData[minDistLine][2], spline.get(minDistLine).heading,spline.get(minDistLine).translationPower,spline.get(minDistLine).rotationPower), targetX,targetY,errorCorrection);
            if(Math.hypot(spline.get(spline.size()-1).X - hardware.getX(), spline.get(spline.size()-1).Y - hardware.getY()) < endMarginOfError){
                splineFinished = true;
            }
            try {
                writer.write("desired X: " + spline.get(minDistLine).X  + ", desired Y: " + spline.get(minDistLine).Y + ", current X: " + hardware.getX()  + ", current Y: " + hardware.getY() +"\n");
            }
            catch(IOException e){
                return;
            }
            parentOP.sleep(5);
        }
        try {
            writer.close();
        }
        catch(IOException e){
            return;
        }
    }
    public double[] getTargetPoint(int currentLine, double percentOfCurrentLineFinished, double lookAheadDist){
        double runningTotal = 0;
        double remainingAmountOfCurrentLine = (1-percentOfCurrentLineFinished) * getLineLength(currentLine);
        runningTotal += remainingAmountOfCurrentLine;
        if(runningTotal > lookAheadDist){
            double lookAheadPercentOfCurrentLine = lookAheadDist / getLineLength(currentLine);
            double[] toReturn = new double[2];
            toReturn[0] = currentLine;
            toReturn[1] = percentOfCurrentLineFinished + lookAheadPercentOfCurrentLine;
            return toReturn;
        }
        else{
            for(int i = currentLine + 1; i < spline.size() -1; i++){
                double runningTotalPrev = runningTotal;
                runningTotal += getLineLength(i);
                if(runningTotal > lookAheadDist){
                    double lookAheadPercentOfCurrentLine = (lookAheadDist-runningTotalPrev) / getLineLength(i);
                    double[] toReturn = new double[2];
                    toReturn[0] = i;
                    toReturn[1] = lookAheadPercentOfCurrentLine;
                    return toReturn;
                }
            }
            double[] toReturn = new double[2];
            toReturn[0] = spline.size()-2;
            toReturn[1] = 1;
            return toReturn;
        }
    }
    public double getLineLength(int linePosition){
        return Math.hypot(spline.get(linePosition).X - spline.get(linePosition + 1).X, spline.get(linePosition).Y - spline.get(linePosition + 1).Y);
    }
    public void setPowersForTargetPoint(Pose currentPoint, double targetX, double targetY, double errorCorrection){
        double targetAngle = Math.atan2(targetY-hardware.getY(),targetX-hardware.getX());
        double currentHeading = hardware.angle;
        double deltaHeading = MathFunctions.keepAngleWithin180Degrees(targetAngle-currentHeading);
        double movementY = currentPoint.translationPower* Math.cos(deltaHeading);
        double movementX = currentPoint.translationPower* Math.sin(deltaHeading);
        double targetAngleToCurrentPathPoint = Math.atan2(currentPoint.Y - hardware.getY(), currentPoint.X-hardware.getX());
        double deltaHeadingToCurrentPathPoint = MathFunctions.keepAngleWithin180Degrees(targetAngleToCurrentPathPoint-currentHeading);
        double distanceToCurrentPathPoint = Math.hypot(currentPoint.Y - hardware.getY(), currentPoint.X - hardware.getX());
        double movementYErrorCorrection = errorCorrection*distanceToCurrentPathPoint* Math.cos(deltaHeadingToCurrentPathPoint);
        double movementXErrorCorrection = errorCorrection*distanceToCurrentPathPoint* Math.sin(deltaHeadingToCurrentPathPoint);
        double deltaHeadingToTurn = MathFunctions.keepAngleWithin180Degrees(currentPoint.heading- currentHeading);
        double turn = currentPoint.rotationPower* Range.clip(deltaHeadingToTurn,-1,1);
        hardware.mecanumDrive.setPowers(movementX+movementXErrorCorrection,movementY+movementYErrorCorrection,turn);
    }
}