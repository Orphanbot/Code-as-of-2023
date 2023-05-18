package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Line Up Test")

public class Line_up extends LinearOpMode {

    HolicronHardware robot = new HolicronHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double DRIVE_SPEED = 0.25;
    static double DRIVE_SPEED_slow = 0.1;
    static double movement = 0;

    static final double COUNTS_PER_MOTOR_REV_1620 = 103.8;    // eg: gobilda 5202 series 1620rpm Motor Encoder
    static final double WORM_GEAR_REDUCTION = 24.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV_1620 * WORM_GEAR_REDUCTION / 360; //counts per degree rotation on worm

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.fiveturn_one.setPosition(0.17);
            robot.fiveturn_two.setPosition(0.17);
            robot.fiveturn_three.setPosition(0.83);
            robot.fiveturn_four.setPosition(0.83);
            sleep(1000);
            lineup();
            robot.clawR.setPosition(0.18);
            robot.clawL.setPosition(0.38);
            break;
        }
    }

    public void lineup() {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (robot.pole_sensor.getDistance(DistanceUnit.MM) > 270){
                while (robot.pole_sensor.getDistance(DistanceUnit.MM) > 270) {
                    encoderDrive(DRIVE_SPEED_slow, 1, 1, 1, 1, 1);
                    telemetry.addData("Distance from pole: ", robot.pole_sensor.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            }
            telemetry.addData("Status", "in line");
            telemetry.update();
            encoderDrive(DRIVE_SPEED_slow, 1.0, 1.0, 1.0, 1.0, 3);
            sleep(750);
            movement = (robot.pole_sensor.getDistance(DistanceUnit.MM) - 165)/25.4;
            encoderDrive(DRIVE_SPEED_slow, -movement, movement, movement, -movement, 3);
            sleep(750);
        }
    }
    public void encoderDrive(double speed,
                             double FrontLeftInches, double RearLeftInches, double FrontRightInches, double RearRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.FrontLeftDrive.getCurrentPosition() + (int) (FrontLeftInches * COUNTS_PER_INCH);
            newRearLeftTarget = robot.RearLeftDrive.getCurrentPosition() + (int) (RearLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.FrontRightDrive.getCurrentPosition() + (int) (FrontRightInches * COUNTS_PER_INCH);
            newRearRightTarget = robot.RearRightDrive.getCurrentPosition() + (int) (RearRightInches * COUNTS_PER_INCH);
            robot.FrontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.RearLeftDrive.setTargetPosition(newRearLeftTarget);
            robot.FrontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.RearRightDrive.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FrontLeftDrive.setPower(Math.abs(speed));
            robot.RearLeftDrive.setPower(Math.abs(speed));
            robot.FrontRightDrive.setPower(Math.abs(speed));
            robot.RearRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() <
                            timeoutS) &&
                    (robot.FrontLeftDrive.isBusy() && robot.RearLeftDrive.isBusy() && robot.FrontRightDrive.isBusy() && robot.RearRightDrive.isBusy())) {
            }

            // Stop all motion;
            robot.FrontLeftDrive.setPower(0);
            robot.RearLeftDrive.setPower(0);
            robot.FrontRightDrive.setPower(0);
            robot.RearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move
        }
    }
}
