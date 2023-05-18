package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.*;
import java.util.*;

public class Odometry_Hardware {

    //Wheel motors
    public DcMotor Front_Left_Drive;
    public DcMotor Front_Right_Drive;
    public DcMotor Rear_Left_Drive;
    public DcMotor Rear_Right_Drive;

    //Odometers
    public DcMotor Left_Dodo;
    public DcMotor Right_Dodo;
    public DcMotor Back_Dodo;

    private HardwareMap hwMap;

    public void init(HardwareMap aHardwareMap) {
        // Save reference to Hardware map
        hwMap = aHardwareMap;

        // Define and Initialize Motors
        Front_Left_Drive = hwMap.get(DcMotor.class, "front_left_drive");
        Front_Right_Drive = hwMap.get(DcMotor.class, "front_right_drive");
        Rear_Left_Drive = hwMap.get(DcMotor.class, "rear_left_drive");
        Rear_Right_Drive = hwMap.get(DcMotor.class, "rear_right_drive");
        //set motors to move in the correct direction
        Front_Left_Drive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Front_Right_Drive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        Rear_Left_Drive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Rear_Right_Drive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motor
        //make sure motors aren't moving
        Front_Left_Drive.setPower(0);
        Front_Right_Drive.setPower(0);
        Rear_Left_Drive.setPower(0);
        Rear_Right_Drive.setPower(0);
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Front_Left_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_Right_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rear_Left_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rear_Right_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //This is just to separate stuff and make it neat
        Front_Left_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Front_Right_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rear_Left_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rear_Right_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left_Dodo = Rear_Left_Drive;
        Right_Dodo = Rear_Right_Drive;
        Back_Dodo = Front_Right_Drive;

    }

        //constants for Odometry math
        final static double L = 27.5;
        final static double B = 10.8;
        final static double R = 4;
        final static double N = 8192;
        final static double cm_per_tick = 2 * Math.PI * R/N;

        //position tracking current
        public int currentRightPosition= 0;
        public int currentLeftPosition= 0;
        public int currentAuxPosition= 0;

        //position tracking previous
        public int oldRightPosition= 0;
        public int oldLeftPosition= 0;
        public int oldAuxPosition= 0;

        public double posX = 0;
        public double posY = 0;
        public double rot = 0;

        public void odometry() {
            oldRightPosition = currentRightPosition;
            oldLeftPosition = currentLeftPosition;
            oldAuxPosition = currentAuxPosition;

            currentRightPosition = -Right_Dodo.getCurrentPosition();
            currentLeftPosition = Left_Dodo.getCurrentPosition();
            currentAuxPosition = Back_Dodo.getCurrentPosition();

            int dn1 = currentLeftPosition  - oldLeftPosition;
            int dn2 = currentRightPosition - oldRightPosition;
            int dn3 = currentAuxPosition - oldAuxPosition;

            // the robot has moved and turned a tiny bit between two measurements:
            double dtheta = cm_per_tick * ((dn2-dn1) / L);
            double dx = cm_per_tick * ((dn1+dn2) / 2.0);
            double dy = cm_per_tick * (dn3 - (B * ( (dn2-dn1) / L )));



            // small movement of the robot gets added to the field coordinate system:
            double theta = rot+(dtheta/2);
            posX += dx * Math.cos(theta) - dy * Math.sin(theta);
            posY += dx * Math.sin(theta) + dy * Math.cos(theta);
            rot += dtheta;
        }
        public void resetpos (){
            Front_Right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Rear_Left_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Rear_Right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Rear_Left_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Front_Right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Rear_Right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Front_Right_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Rear_Left_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Rear_Right_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

