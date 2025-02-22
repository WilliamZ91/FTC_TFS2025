package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TFS_Auton_2025_Broken", group = "Auto1")
public class Auton2025_test extends LinearOpMode {

    // Drive motors and IMU
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private IMU imu;

    // Servos and slide control
    private Servo specimen;      // For grabbing/releasing the specimen
    private Servo OuttakeWrist;  // Bucket servo
    private Servo IntakeWrist;   // Intake servo
    private CRServo ClawL, ClawR; // Claw servos for horizontal intake

    // ArmExtension for vertical and horizontal slides
    private ArmExtension armExtension;

    // Constants for drive calculations
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_CM = 10.4;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * Math.PI);

    boolean XisPressed = false;

    // Desired heading variable
    private double desiredHeading;

    @Override
    public void runOpMode() {
        initHardware();
        telemetry.addData("Select Mode", "Press X for Sample, Y for Specimen");
        sleep(1000);
        if(gamepad1.x) {
            XisPressed = true;
        } else if (gamepad1.y){
            XisPressed = false;
        }
        telemetry.update();
        // Reset/allow IMU calibration.
        sleep(5000);
        // Store the initial desired heading.
        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw();
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        telemetry.addData("X Sample Pressed?", XisPressed);

        telemetry.update();
        waitForStart();
        // --- Common Preload Specimen Process (Steps 0-7
        // After the common process, allow driver to select the mode:


        if (XisPressed) {
            runCase1();
        } else if (!XisPressed) {
            runCase2();
        }
    }

    // This function encapsulates the common specimen process (preload specimen phase)
    private void specimenProcess() {
        driveStraight(0.5, 10, 5);
        correctHeading(desiredHeading, 2);

        // Step 1: Grab the specimen.
        specimen.setPosition(1);
        sleep(500);

        // Step 2: Lift vertical slide to -0.45.
        Thread liftThread = new Thread(() -> {
            armExtension.Arm_Vertical_Position(-0.445, 0.7);
            while (opModeIsActive() &&
                    Math.abs(armExtension.getCurrentVerticalLength() - (-0.445)) > 0.01) {
                sleep(10);
            }
        });
        liftThread.start();
        sleep(500);

        // Step 3: Drive forward (tuned value -100 cm).
        driveStraight(0.5, -70, 5);
        try { liftThread.join(); } catch (InterruptedException e) { }
        sleep(500);

        // Step 4: Lower vertical slide slightly to -0.29.
        driveStraight(0.7,-10,5);
        armExtension.Arm_Vertical_Position(-0.265, 1);
        while (opModeIsActive() &&
                Math.abs(armExtension.getCurrentVerticalLength() - (-0.265)) > 0.01) {
            sleep(10);
        }
        sleep(600);

        // Step 5: Open the specimen servo to 0.4.


        // Step 6: Lower vertical slide completely to 0.
        armExtension.Arm_Vertical_Position(0, 0.7);
        while (opModeIsActive() &&
                Math.abs(armExtension.getCurrentVerticalLength() - 0) > 0.01) {
            sleep(10);
        }
        specimen.setPosition(0.4);
        sleep(500);
        sleep(200);

        // Step 7: Drive backward 30 cm (tuned value: 40 cm here).
        driveStraight(0.5, 40, 5);
        correctHeading(desiredHeading, 2);
    }

    // CASE 1: Triggered by gamepad1.x – Sample
    private void runCase1() {
        strafeRight(0.5, 60, 3);
        correctHeading(desiredHeading, 2);
        sleep(300);
        specimenProcess();
        // Strafe left.
        strafeRight(0.5, -40, 3);
        // Put bucket into traverse position.
        OuttakeWrist.setPosition(0.17);
        sleep(200);
        // Drive backward.
        driveStraight(0.5, 40, 5);
        // Strafe right.
        strafeRight(0.5, 40, 3);
        // Lift the climb (vertical slide) up until it can hit the first pole.
        armExtension.Arm_Vertical_Position(1.0, 0.7);
        while (opModeIsActive() && Math.abs(armExtension.getCurrentVerticalLength() - 1.0) > 0.01) {
            sleep(10);
        }
        // Stop all motors.
        stopMotors();
    }

    // CASE 2: Triggered by gamepad1.y – Specimen
    private void runCase2() {
        strafeRight(0.5, -10, 3);
        sleep(300);
        specimenProcess();
//        // Step A: Strafe right.
        strafeRight(0.5, 60, 3);
        sleep(200);
        driveStraight(0.5, 40, 5);
        sleep(100);

//
//        // Step B: Turn 180°.
//        leftFront.setPower(1);
//        leftRear.setPower(1);
//        rightFront.setPower(-1);
//        rightRear.setPower(-1);
//        sleep(250);
//        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw();
//        correctHeading(desiredHeading, 2);
//
//        // Step C: Set bucket to traverse position.
//        OuttakeWrist.setPosition(0.17);
//        sleep(200);

        // Step D: Execute horizontal intake process.
//        Thread intakeThread = new Thread(() -> {
//            ClawL.setPower(0);
//            ClawR.setPower(0);
//            IntakeWrist.setPosition(0.50);
//            sleep(50);
//            armExtension.Arm_Horizontal_Position(0.340, 0.9);
//            while (opModeIsActive() &&
//                    Math.abs(armExtension.getCurrentHorizontalLength() - 0.340) > 0.01) {
//                sleep(10);
//            }
//            OuttakeWrist.setPosition(0.0);
//            IntakeWrist.setPosition(0.90);
//            sleep(900);
//            ClawL.setPower(-1);
//            ClawR.setPower(1);
//            sleep(700);
//            IntakeWrist.setPosition(0.52);
//            sleep(500);
//            ClawL.setPower(0);
//            ClawR.setPower(0);
//            armExtension.Arm_Horizontal_Position(0.170, 0.9);
//            while (opModeIsActive() &&
//                    Math.abs(armExtension.getCurrentHorizontalLength() - 0.170) > 0.01) {
//                sleep(10);
//            }
//        });
//        intakeThread.start();
//        try { intakeThread.join(); } catch (InterruptedException e) { }

        // Step F: Turn 180° again.
//        turnRight(0.5, 180, 3);
//        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw();
//        correctHeading(desiredHeading, 2);
//        sleep(200);
//
//        // Step G: Move forward a little bit.
//        driveStraight(0.5, 20, 5);
//        sleep(200);
//
//        // Step H: Spit out the sample (assume setting OuttakeWrist to 0.4 spitting out).
//        OuttakeWrist.setPosition(0.4);
//        sleep(500);
//
//        // Step I: Move forward a little bit.
//        driveStraight(0.5, -20, 5);
//        sleep(200);
//
//        // Step J: Turn 180° again.
//        turnRight(0.5, 180, 3);
//        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw();
//        correctHeading(desiredHeading, 2);
//        sleep(200);
//        // Step E: Raise vertical slide a little bit.
//        armExtension.Arm_Vertical_Position(-0.1, 0.7);
//        while (opModeIsActive() && Math.abs(armExtension.getCurrentVerticalLength() - (-0.1)) > 0.01) {
//            sleep(10);
//        }
//        sleep(200);
//        specimen.setPosition(0.4);
//
//        // Step K: Drive forward into the wall.
//        driveStraight(0.5, -30, 5);
//        sleep(400);

//        specimen.setPosition(1);
//
//        // Step L: Drive backward a little bit.
//        driveStraight(0.5, 40, 5);
//        sleep(200);
//
//        // Step M: Strafe left, then drive backward into the submersible.
//        strafeRight(0.5, -20, 3);
//        sleep(200);
//
//        // Step N: Repeat the specimen process (common preload phase).
//        specimenProcess();
//
//        // Step O: Then move forward, strafe to the right, and then drive backward all the way to park.
//        driveStraight(0.5, 20, 5);
//        strafeRight(0.5, 50, 3);
//        driveStraight(0.5, 40, 5);



        // Finally, stop all motors.
        stopMotors();
    }

    // --- Heading Correction Routine ---
    // Uses a proportional controller to adjust the robot's heading to the desired value.
    private void correctHeading(double desired, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double kP = 0.02; // Adjust as needed.
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            double current = imu.getRobotYawPitchRollAngles().getYaw();
            double error = desired - current;
            telemetry.addData("Heading Correction", "Current: %.2f, Desired: %.2f, Error: %.2f", current, desired, error);
            telemetry.update();
            if (Math.abs(error) < 1.0) break;
            double turnPower = Math.abs(error) * kP;
            turnPower = Math.max(0.2, Math.min(turnPower, 0.5));
            if (error > 0) {
                // Turn right
                leftFront.setPower(turnPower);
                leftRear.setPower(turnPower);
                rightFront.setPower(-turnPower);
                rightRear.setPower(-turnPower);
            } else {
                // Turn left
                leftFront.setPower(-turnPower);
                leftRear.setPower(-turnPower);
                rightFront.setPower(turnPower);
                rightRear.setPower(turnPower);
            }
            sleep(50);
        }
        stopMotors();
        telemetry.addData("Heading Correction", "Complete");
        telemetry.update();
        sleep(100);
    }

    // --- Driving/Strafing/Turning Methods (unchanged tuning) ---
    private void driveStraight(double power, double cm, double timeout) {
        int targetPosition = (int)(cm * COUNTS_PER_CM);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + targetPosition);
        setRunToPositionMode();

        setAllMotorPower(Math.abs(power));

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeout && motorsAreBusy()) {
            telemetry.addData("Driving", "Target: %d", targetPosition);
            telemetry.update();
        }
        stopMotors();
        setRunUsingEncoderMode();
    }

    /**
     * Strafes right for a given distance in cm.
     * (Use a negative distance to strafe left.)
     */
    private void strafeRight(double power, double cm, double timeout) {
        int target = (int)(cm * COUNTS_PER_CM);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
        setRunToPositionMode();

        setAllMotorPower(Math.abs(power));

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeout &&
                leftFront.isBusy() && leftRear.isBusy() &&
                rightFront.isBusy() && rightRear.isBusy()) {
            telemetry.addData("Strafing", "Target: %d", target);
            telemetry.update();
        }
        stopMotors();
        setRunUsingEncoderMode();
    }

    // Turn right (clockwise) by a specified number of degrees using a proportional controller.
    // (This routine is as per your tested turning routine.)
    private void turnRight(double power, double degrees, double timeout) {
        setRunUsingEncoderMode();
        double initialAngle = imu.getRobotYawPitchRollAngles().getYaw();
        double targetAngle = initialAngle + degrees;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeout) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();
            double error = targetAngle - currentAngle;
            telemetry.addData("TurnRight", "Target: %.2f, Current: %.2f, Error: %.2f", targetAngle, currentAngle, error);
            telemetry.update();
            if (Math.abs(error) < 1.0) break;
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(-power);
            rightRear.setPower(-power);
            sleep(50);
        }
        stopMotors();
        sleep(100);
    }

    private void setAllMotorPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    private void stopMotors() {
        setAllMotorPower(0);
    }

    private void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunUsingEncoderMode();
    }

    private void setRunToPositionMode() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setRunUsingEncoderMode() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean motorsAreBusy() {
        return leftFront.isBusy() && leftRear.isBusy() &&
                rightFront.isBusy() && rightRear.isBusy();
    }
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

        // Initialize the IMU with proper orientation.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        // Optionally wait for calibration here if needed.
        sleep(500);

        resetEncoders();

        armExtension = new ArmExtension();
        armExtension.initArmExtensionHardware(hardwareMap);
        armExtension.setOpMode(this);  // Pass this opmode so ArmExtension can check opModeIsActive()

        specimen = hardwareMap.get(Servo.class, "specimen");
        OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");
        ClawL = hardwareMap.get(CRServo.class, "ClawL");
        ClawR = hardwareMap.get(CRServo.class, "ClawR");

        specimen.setPosition(1);
        OuttakeWrist.setPosition(0.17);
        IntakeWrist.setPosition(0.5);
    }
}
