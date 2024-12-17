package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TFS_Auton_2025_Updated", group = "Auto1")
public class Auton2025_test extends LinearOpMode {

    // Motors and IMU
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private IMU imu;

    // Constants for motion
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_CM = 10.4;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        telemetry.addData("Status", "Ready to Start");
        telemetry.update();
        waitForStart();

        // Autonomous Sequence
        driveStraight(0.5, 50, 5); // Drive forward 50 cm
        turnIMU(0.3, 90);          // Turn 90 degrees clockwise
        driveStraight(0.5, -30, 3); // Drive backward 30 cm
    }

    /**
     * Initializes motors and IMU with proper settings.
     */
    private void initHardware() {
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // IMU orientation setup
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        resetEncoders();
    }

    /**
     * Drive the robot straight for a given distance.
     *
     * @param power  Speed of movement (0 to 1).
     * @param cm     Distance in cm (positive = forward, negative = backward).
     * @param timeout Timeout in seconds.
     */
    private void driveStraight(double power, double cm, double timeout) {
        int targetPosition = (int) (cm * COUNTS_PER_CM);

        // Set target positions
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + targetPosition);

        setRunToPositionMode();
        setAllMotorPower(Math.abs(power));

        // Wait until the motors reach the position or timeout occurs
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeout &&
                motorsAreBusy()) {
            telemetry.addData("Driving", "Target: %d", targetPosition);
            telemetry.update();
        }
        stopMotors();
        setRunUsingEncoderMode();
    }

    /**
     * Turn the robot a specific angle using IMU.
     *
     * @param power Speed of turning (0 to 1).
     * @param angle Target angle to turn (positive = clockwise, negative = counterclockwise).
     */
    private void turnIMU(double power, double angle) {
        double initialAngle = imu.getRobotYawPitchRollAngles().getYaw();
        double targetAngle = initialAngle + angle;
        telemetry.addData("Initial Yaw", initialAngle);
        telemetry.update();


        while (opModeIsActive()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();

            double error = targetAngle - currentAngle;
            error = normalizeAngle(error);

            if (Math.abs(error) <= 1.0) break;

            double turnPower = power * Math.signum(error);

            // Apply turning power
            leftFront.setPower(turnPower);
            leftRear.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightRear.setPower(-turnPower);

            telemetry.addData("Turning", "Target: %.1f, Current: %.1f", targetAngle, currentAngle);
            telemetry.update();
        }
        stopMotors();
    }

    private double normalizeAngle(double angle){
        while (angle > 180) angle-= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /**
     * Helper function to stop all motors.
     */
    private void stopMotors() {
        setAllMotorPower(0);
    }

    /**
     * Helper function to set the same power for all motors.
     */
    private void setAllMotorPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    /**
     * Helper function to reset motor encoders.
     */
    private void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setRunUsingEncoderMode();
    }

    /**
     * Helper function to set motors to RUN_TO_POSITION mode.
     */
    private void setRunToPositionMode() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Helper function to set motors to RUN_USING_ENCODER mode.
     */
    private void setRunUsingEncoderMode() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Helper function to check if all motors are busy.
     */
    private boolean motorsAreBusy() {
        return leftFront.isBusy() && leftRear.isBusy() &&
                rightFront.isBusy() && rightRear.isBusy();
    }
}
