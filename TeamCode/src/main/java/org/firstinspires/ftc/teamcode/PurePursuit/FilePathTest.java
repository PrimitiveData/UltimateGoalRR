package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

@TeleOp(name="FilePathTest", group="TeleOp")
public class FilePathTest extends LinearOpMode {
    public void runOpMode(){
        File file = new File("//sdcard//FIRST//points.txt.txt");
        telemetry.addLine(file.getAbsolutePath());
        telemetry.update();
        sleep(1000);
        Scanner scnr;
        try {
             scnr = new Scanner(file);
        }
        catch(FileNotFoundException e){
            telemetry.addLine("Bruh, file not found");
            telemetry.update();
            sleep(5000);
            return;
        }
        waitForStart();
        telemetry.addLine(scnr.nextLine());
        telemetry.update();
        FileWriter writer;
        try {
             writer = new FileWriter("//sdcard//FIRST//points.txt.txt", true);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        try {
            writer.write("herro");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
