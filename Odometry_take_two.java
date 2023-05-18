package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.index.qual.Positive;


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

@TeleOp(name="DodoTracking", group="Linear Opmode")
//@Disabled
public class Odometry_take_two extends LinearOpMode {

    Odometry_Hardware robot           = new Odometry_Hardware();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    // DigitalChannel digitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {

        double POWER = 0.7;
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

            robot.odometry();
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            forward = -gamepad1.left_stick_y;
            strafe =  gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;

            front_left = forward + clockwise + strafe;
            front_right = forward - clockwise - strafe;
            rear_left = forward + clockwise - strafe;
            rear_right = forward - clockwise + strafe;

            front_left    = Range.clip(front_left, -1, 1) ;
            front_right   = Range.clip(front_right, -1, 1) ;
            rear_left    = Range.clip(rear_left, -1, 1) ;
            rear_right   = Range.clip(rear_right, -1, 1) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.Front_Left_Drive.setPower(front_left * POWER);
            robot.Front_Right_Drive.setPower(front_right * POWER);
            robot.Rear_Left_Drive.setPower(rear_left * POWER);
            robot.Rear_Right_Drive.setPower(rear_right * POWER);

            if (gamepad1.right_bumper) {
                POWER = 1.0;
            } else {
                POWER = 0.7;
            }

            if (gamepad1.left_bumper) {
                robot.resetpos();
            }

            if ((gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_x > -0.2) &&
                    (gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2) &&
                    (gamepad1.left_stick_y  < 0.2 && gamepad1.left_stick_y > -0.2)){

                POWER = 0;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position", "X: " + robot.posX);
            telemetry.addData("Position", "Y: " + robot.posY);
            telemetry.addData("Rotation", "Theta: " + Math.toDegrees(robot.rot));
            telemetry.addData("motor powers", " " + front_left + front_right);
            telemetry.addData("motor powers", " " + rear_left + rear_right);
            telemetry.update();
        }
    }
}
