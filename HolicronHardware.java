package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HolicronHardware {
    /* Public OpMode members */
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor RearLeftDrive = null;
    public DcMotor RearRightDrive = null;
    public Servo fiveturn_one = null;
    public Servo fiveturn_two = null;
    public Servo fiveturn_three = null;
    public Servo fiveturn_four = null;
    public Servo clawR = null;
    public Servo clawL = null;
    public DistanceSensor pole_sensor = null;
    //public CRServo Arm = null;
    //public CRServo Aim = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HolicronHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontLeftDrive = hwMap.get(DcMotor.class, "front_left_drive");
        FrontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        RearLeftDrive = hwMap.get(DcMotor.class, "rear_left_drive");
        RearRightDrive = hwMap.get(DcMotor.class, "rear_right_drive");
        fiveturn_one  = hwMap.get(Servo.class, "5turn1");
        fiveturn_two  = hwMap.get(Servo.class, "5turn2");
        fiveturn_three  = hwMap.get(Servo.class, "5turn3");
        fiveturn_four  = hwMap.get(Servo.class, "5turn4");
        clawR  = hwMap.get(Servo.class, "claw_right");
        clawL  = hwMap.get(Servo.class, "claw_left");
        pole_sensor = hwMap.get(DistanceSensor.class, "pole_sensor");
        //Arm  = hwMap.get(CRServo.class, "arm");
        //Aim  = hwMap.get(CRServo.class, "aim");

        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        RearLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RearRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motor

        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //This is just to separate stuff and make it neat

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawR.setPosition(0.42);
        clawL.setPosition(0.62);
        fiveturn_one.setPosition(0.97);
        fiveturn_two.setPosition(0.97);
        fiveturn_three.setPosition(0.03);
        fiveturn_four.setPosition(0.03);


    }
}
