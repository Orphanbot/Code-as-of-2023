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

@Autonomous(name="Right Side Auto")

public class ORSide extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    HolicronHardware robot = new HolicronHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double DRIVE_SPEED = 0.25;
    static double DRIVE_SPEED_slow = 0.1;

    static final double COUNTS_PER_MOTOR_REV_1620 = 103.8;    // eg: gobilda 5202 series 1620rpm Motor Encoder
    static final double WORM_GEAR_REDUCTION = 24.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV_1620 * WORM_GEAR_REDUCTION / 360; //counts per degree rotation on worm
    static double movement = 0;

    //Green
    //old range, keep just in case new don't work
    private double crThreshHigh = 119;
    private double crThreshLow = 19;
    private double cbThreshHigh = 119;
    private double cbThreshLow = 24; //increase range

    public static Scalar scalarLowerYCrCb = new Scalar(26.0, 19.0, 24.0);
    public static Scalar scalarUpperYCrCb = new Scalar(225.0, 123.0, 123.0);

    //Magenta
    //private double crThreshHigh = 240;
    //private double crThreshLow = 137;
    //private double cbThreshHigh = 240;
    //private double cbThreshLow = 127; //increase range

    //public static Scalar scalarLowerYCrCb = new Scalar(0.0, 137.0, 127.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(235.0, 240.0, 240.0);

    //Cyan
    //private double crThreshHigh = 71;
    //private double crThreshLow = 16;
    //private double cbThreshHigh = 206;
    //private double cbThreshLow = 126; //increase range

    //public static Scalar scalarLowerYCrCb = new Scalar(128.0, 16.0, 126.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(235.0, 71.0, 206.0);

    private int minRectangleArea = 20000;

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Only if you are using ftcdashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        FtcDashboard.getInstance().startCameraStream(webcam, 10);

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
            if (pipeline.error) {
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            // testing(pipeline);

            // Watch our YouTube Tutorial for the better explanation
            double rectangleArea = pipeline.getRectArea();

            //Check to see if the rectangle has a large enough area to be a marker.
            if (rectangleArea > 500 && rectangleArea < 4000) {

                telemetry.addData("code found", "two");
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.update();
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
                sleep(100);
                robot.fiveturn_one.setPosition(0.965);
                robot.fiveturn_two.setPosition(0.965);
                robot.fiveturn_three.setPosition(0.035);
                robot.fiveturn_four.setPosition(0.035);
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 10.0);
                encoderDrive(DRIVE_SPEED, -26, 26, 26, -26, 10.0);
                encoderDrive(DRIVE_SPEED, 48, 48, 48, 48, 10.0);
                encoderDrive(DRIVE_SPEED, 11, -11, -11, 11, 10.0);
                robot.fiveturn_one.setPosition(0.17);
                robot.fiveturn_two.setPosition(0.17);
                robot.fiveturn_three.setPosition(0.83);
                robot.fiveturn_four.setPosition(0.83);
                sleep(1000);
                lineup();
                robot.fiveturn_one.setPosition(0.36);
                robot.fiveturn_two.setPosition(0.36);
                robot.fiveturn_three.setPosition(0.64);
                robot.fiveturn_four.setPosition(0.64);
                robot.clawR.setPosition(0.18);
                robot.clawL.setPosition(0.38);
                sleep(250);
                encoderDrive(DRIVE_SPEED_slow, -7, -7, -7, -7, 10.0);
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
                sleep(250);
                robot.fiveturn_one.setPosition(1.00);
                robot.fiveturn_two.setPosition(1.00);
                robot.fiveturn_three.setPosition(0.00);
                robot.fiveturn_four.setPosition(0.00);
                sleep(250);
                encoderDrive(DRIVE_SPEED, 13, -13, -13, 13, 10.0);
                encoderDrive(DRIVE_SPEED, -20, -20, -20, -20, 10.0);
                sleep(20000);
                break;
            } else if (rectangleArea > 4600 && rectangleArea < 7000) {

                telemetry.addData("code found", "one");
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.update();
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
                sleep(100);
                robot.fiveturn_one.setPosition(0.965);
                robot.fiveturn_two.setPosition(0.965);
                robot.fiveturn_three.setPosition(0.035);
                robot.fiveturn_four.setPosition(0.035);
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 10.0);
                encoderDrive(DRIVE_SPEED, -26, 26, 26, -26, 10.0);
                encoderDrive(DRIVE_SPEED, 48, 48, 48, 48, 10.0);
                encoderDrive(DRIVE_SPEED, 11, -11, -11, 11, 10.0);
                robot.fiveturn_one.setPosition(0.17);
                robot.fiveturn_two.setPosition(0.17);
                robot.fiveturn_three.setPosition(0.83);
                robot.fiveturn_four.setPosition(0.83);
                sleep(1000);
                lineup();
                robot.fiveturn_one.setPosition(0.36);
                robot.fiveturn_two.setPosition(0.36);
                robot.fiveturn_three.setPosition(0.64);
                robot.fiveturn_four.setPosition(0.64);
                robot.clawR.setPosition(0.18);
                robot.clawL.setPosition(0.38);
                sleep(250);
                encoderDrive(DRIVE_SPEED_slow, -7, -7, -7, -7, 10.0);
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
                sleep(250);
                robot.fiveturn_one.setPosition(1.00);
                robot.fiveturn_two.setPosition(1.00);
                robot.fiveturn_three.setPosition(0.00);
                robot.fiveturn_four.setPosition(0.00);
                sleep(250);
                encoderDrive(DRIVE_SPEED, -13, 13, 13, -13, 10.0);
                encoderDrive(DRIVE_SPEED, -20, -20, -20, -20, 10.0);
                sleep(20000);
                break;
            }else {

                telemetry.addData("code found", "three");
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.update();
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
                sleep(100);
                robot.fiveturn_one.setPosition(0.965);
                robot.fiveturn_two.setPosition(0.965);
                robot.fiveturn_three.setPosition(0.035);
                robot.fiveturn_four.setPosition(0.035);
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 10.0);
                encoderDrive(DRIVE_SPEED, -26, 26, 26, -26, 10.0);
                encoderDrive(DRIVE_SPEED, 48, 48, 48, 48, 10.0);
                encoderDrive(DRIVE_SPEED, 11, -11, -11, 11, 10.0);
                robot.fiveturn_one.setPosition(0.17);
                robot.fiveturn_two.setPosition(0.17);
                robot.fiveturn_three.setPosition(0.83);
                robot.fiveturn_four.setPosition(0.83);
                sleep(1000);
                lineup();
                robot.fiveturn_one.setPosition(0.36);
                robot.fiveturn_two.setPosition(0.36);
                robot.fiveturn_three.setPosition(0.64);
                robot.fiveturn_four.setPosition(0.64);
                robot.clawR.setPosition(0.18);
                robot.clawL.setPosition(0.38);
                sleep(250);
                encoderDrive(DRIVE_SPEED_slow, -7, -7, -7, -7, 10.0);
                robot.clawR.setPosition(0.42);
                robot.clawL.setPosition(0.62);
                sleep(250);
                robot.fiveturn_one.setPosition(1.00);
                robot.fiveturn_two.setPosition(1.00);
                robot.fiveturn_three.setPosition(0.00);
                robot.fiveturn_four.setPosition(0.00);
                sleep(250);
                encoderDrive(DRIVE_SPEED, 39, -39, -39, 39, 10.0);
                encoderDrive(DRIVE_SPEED, -20, -20, -20, -20, 10.0);
                sleep(20000);
                break;
            }
        }
    }

    public void testing(ContourPipeline pipeline) {
        if (lowerRuntime + 0.05 < getRuntime()) {
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if (upperRuntime + 0.05 < getRuntime()) {
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }

    public double inValues(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
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

            sleep(250);   // optional pause after each move
        }
    }
    public void lineup() {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (robot.pole_sensor.getDistance(DistanceUnit.MM) > 270){
                while (robot.pole_sensor.getDistance(DistanceUnit.MM) > 270 && opModeIsActive()) {
                    encoderDrive(DRIVE_SPEED_slow, 1, 1, 1, 1, 1);
                    telemetry.addData("Distance from pole: ", robot.pole_sensor.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            }
            telemetry.addData("Status", "in line");
            telemetry.update();
            encoderDrive(DRIVE_SPEED_slow, 1.0, 1.0, 1.0, 1.0, 3);
            sleep(750);
            movement = (robot.pole_sensor.getDistance(DistanceUnit.MM) - 155)/25.4;
            encoderDrive(DRIVE_SPEED_slow, -movement, movement, movement, -movement, 3);
            sleep(750);
        }
    }
}