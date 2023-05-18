package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Holicron 2P", group="Linear Opmode")
//@Disabled
public class HolicronBot_2P extends LinearOpMode {

    HolicronHardware robot           = new HolicronHardware();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    // DigitalChannel digitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {

        double POWER = 0.7;
        double height = 0.0;
        double Servo_Position1 = 1.00;
        double Servo_Position2 = 0.00;
        double front_left;
        double front_right;
        double rear_left;
        double rear_right;
        double forward; // push joystick1 forward to go forward
        double strafe; // push joystick1 to the right to strafe right
        double clockwise; // push joystick2 to the right to rotate clockwise

        // get a reference to our digitalTouch object.
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            forward = -gamepad1.left_stick_y;
            strafe =  gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;

            front_left = forward - clockwise + strafe;
            front_right = forward + clockwise - strafe;
            rear_left = forward - clockwise - strafe;
            rear_right = forward + clockwise + strafe;

            front_left    = Range.clip(front_left, -1, 1) ;
            front_right   = Range.clip(front_right, -1, 1) ;
            rear_left    = Range.clip(rear_left, -1, 1) ;
            rear_right   = Range.clip(rear_right, -1, 1) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.FrontLeftDrive.setPower(front_left * POWER);
            robot.FrontRightDrive.setPower(front_right * POWER);
            robot.RearLeftDrive.setPower(rear_left * POWER);
            robot.RearRightDrive.setPower(rear_right * POWER);

            if (gamepad1.right_bumper) {
                POWER = 1.0;
            } else if(height == 1.0) {
                POWER = 0.24;
            } else if(height == 0.7) {
                POWER = 0.34;
            } else if(height == 0.3) {
                POWER = 0.44;
            } else {
                POWER = 0.55;
            }

            if ((gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) &&
                    (gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2) &&
                    (gamepad1.left_stick_y  < 0.2 && gamepad1.left_stick_y > -0.2)){

                POWER = 0;
            }

            if (gamepad2.dpad_up) {
                Servo_Position1 = 0.16;
                Servo_Position2 = 0.84;
            }
            if (gamepad2.dpad_right) {
                Servo_Position1 = 0.48;
                Servo_Position2 = 0.52;
            }
            if (gamepad2.dpad_left) {
                Servo_Position1 = 0.69;
                Servo_Position2 = 0.31;
            }
            if (gamepad2.dpad_down) {
                Servo_Position1 = 1.00;
                Servo_Position2 = 0.00;
            }

            if (gamepad2.y) {
                Servo_Position1 = Servo_Position1 - 0.008;
                Servo_Position2 = Servo_Position2 + 0.008;
            }
            if (gamepad2.x) {
                Servo_Position1 = Servo_Position1 + 0.008;
                Servo_Position2 = Servo_Position2 - 0.008;
            }

            Servo_Position1 = Range.clip(Servo_Position1, 0.16, 1.00);
            Servo_Position2 = Range.clip(Servo_Position2, 0.00, 0.84);

            if (Servo_Position1 <= 1.00 && Servo_Position1 >= 0.80){
                height = 0;
            } else if (Servo_Position1 <= 0.79 && Servo_Position1 >= 0.59){
                height = 0.3;
            } else if (Servo_Position1 <= 0.58 && Servo_Position1 >= 0.32){
                height = 0.7;
            } else if (Servo_Position1 <= 0.31 && Servo_Position1 >= 0.15){
                height = 1.0;
            }

            robot.fiveturn_one.setPosition(Servo_Position1);
            robot.fiveturn_two.setPosition(Servo_Position1);
            robot.fiveturn_three.setPosition(Servo_Position2);
            robot.fiveturn_four.setPosition(Servo_Position2);

            if (gamepad2.b) {
                robot.clawR.setPosition(0.18);
                robot.clawL.setPosition(0.38);
            }
            if (gamepad2.a) {
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("servo upper left: ", robot.fiveturn_one.getPosition());
            telemetry.addData("Servo lower right: ", robot.fiveturn_two.getPosition());
            telemetry.addData("Servo upper right: ", robot.fiveturn_three.getPosition());
            telemetry.addData("Servo lower left: ", robot.fiveturn_four.getPosition());
            telemetry.addData("Motors", "front_left: " + front_left);
            telemetry.addData("Motors", "front_right: " + front_right);
            telemetry.addData("Motors", "rear_left: " + rear_left);
            telemetry.addData("Motors", "rear_right: " + rear_right);
            telemetry.update();
        }
    }
}
