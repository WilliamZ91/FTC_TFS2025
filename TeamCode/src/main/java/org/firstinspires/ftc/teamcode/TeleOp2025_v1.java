package org.firstinspires.ftc.teamcode;

import android.graphics.SweepGradient;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name = "TFS_TeleOp2025", group = "FTC2025")
public class TeleOp2025_v1 extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;
    //double targetAngle = Math.toRadians(90 * TFS_Auton_2024.blueVal);
    ArmSystem armSystem;
    private Servo IntakeWrist = null;
    private Servo OuttakeWrist = null;

    ColorSensor colorSensor;

    private CRServo ClawL = null;
    private CRServo ClawR = null;
    private Servo specimen;


    private double speedfactor = 0.4;
    private double imuAngle = 0.0;

    IMU imu;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotorEx climb;
    private boolean autoThreadFlag = false;
    private boolean traverseMode = false;

    private final int CLIMB_MAX_POSITION = 9500; // fully extended (all the way down)
    private final int CLIMB_MIN_POSITION = 0;     // fully retracted (all the way up)
    // Conversion factor for encoder ticks to distance (e.g., cm)
    private final double TICKS_PER_CM = 100.0; // adjust this conversion factor as needed


    private int bButton_DelayCnt = 0;
    private ArmExtension armExtension;
    double position = 0;
    boolean isClawActive = false;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        climb = hardwareMap.get(DcMotorEx.class, "climb");

        //        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        DcMotor Arm_HorizontalMotor = hardwareMap.get(DcMotor.class, "laterator");
        DcMotor Arm_VerticalMotor = hardwareMap.get(DcMotor.class, "armL");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

//        specimen.setDirection(Servo.Direction.REVERSE);


        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //Need to put armRight and armLeft in hardware map, for armsystem code
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive.setPoseEstimate(PoseStorage.currentPose);
        //DcMotor armLeftMotor = hardwareMap.get(DcMotor.class, "armLeft");
        OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");
        specimen = hardwareMap.get(Servo.class, "specimen");
        ClawL = hardwareMap.get(CRServo.class, "ClawL");
        ClawR = hardwareMap.get(CRServo.class, "ClawR");

        armExtension = new ArmExtension();
        armExtension.initArmExtensionHardware(hardwareMap);
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        OuttakeWrist.setPosition(0.17);//init
        sleep(1000);
        IntakeWrist.setPosition(0.5);
        waitForStart();
        position = 0;

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //armSystem.armTeleOp(gamepad2);
            //ArmExtendHardware.(gamepad2.right_stick_y);
            //Pose2d poseEstimate = drive.getPoseEstimate();

            // Update everything. Odometry. Etc.
            // drive.update();
            double max;

            double speedFactor;
            if (gamepad1.right_bumper) {
                speedFactor = 0.9;
            } else {
                speedFactor = 0.4;
            }

            // Control horizontal slide (laterator) with right joystick (horizontal)
            armExtension.controlHorizontalExtend(-gamepad2.right_stick_y);

            // Control vertical slide with left joystick (vertical)
            armExtension.controlVerticalExtend(gamepad2.left_stick_y);

            // Add telemetry to show the length of both slides


            // now the orientation of robot is changed
            double leftStickXPos = -gamepad1.left_stick_x * speedFactor;
            double leftStickYPos = -gamepad1.left_stick_y * speedFactor;
            double rightStickXPos = gamepad1.right_stick_x * speedFactor;

            double denominator = Math.max(Math.abs(leftStickYPos) + Math.abs(leftStickXPos) + Math.abs(rightStickXPos), 1);

            double lfPower = (leftStickYPos - leftStickXPos - rightStickXPos) / denominator;
            double lrPower = (leftStickYPos + leftStickXPos - rightStickXPos) / denominator;
            double rfPower = (leftStickYPos + leftStickXPos + rightStickXPos) / denominator;
            double rrPower = (leftStickYPos - leftStickXPos + rightStickXPos) / denominator;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
            max = Math.max(max, Math.abs(lrPower));
            max = Math.max(max, Math.abs(rrPower));

            if (max > 1.0) {
                lfPower /= max;
                rfPower /= max;
                lrPower /= max;
                rrPower /= max;
            }

            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftRear.setPower(lrPower);
            rightRear.setPower(rrPower);


            if (!armExtension.HorArmFlag && gamepad2.left_bumper && !autoThreadFlag) {
                autoThreadFlag = true;
                armExtension.HorArmFlag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        ClawL.setPower(0);
                        ClawR.setPower(0);
                        IntakeWrist.setPosition(0.50); // Neutral position
                        sleep(50);
                        armExtension.Arm_Horizontal_Position(0.170, 0.9);
                        OuttakeWrist.setPosition(0);//intake
                        sleep(1300);
                        IntakeWrist.setPosition(0.20);
                        sleep(900);
                        ClawL.setPower(1);
                        ClawR.setPower(-1);
                        sleep(700);
                        IntakeWrist.setPosition(0.5);
                        sleep(1000);
                        armExtension.HorArmFlag = false;
                        autoThreadFlag = false;
                        while (opModeIsActive() && Math.abs(armExtension.getCurrentHorizontalLength() - 0.170) > 0.01) {
                            sleep(10);
                        }
                        ClawL.setPower(0);
                        ClawR.setPower(0);
                    }
                }).start();
            }

            if (gamepad2.right_bumper) {
                OuttakeWrist.setPosition(0.17);//init
            }

            // DPAD FUNCTION
            //----------------------------------------------------------------

            if (gamepad2.dpad_up) { // Outtake
                ClawL.setPower(1);
                ClawR.setPower(-0.9);
            }

            if (gamepad2.dpad_down) { // Intake
                ClawL.setPower(-1);
                ClawR.setPower(0.9);
            }

            if (gamepad2.dpad_left) { // Outtake
                IntakeWrist.setPosition(0.15);
            }

            if (gamepad2.dpad_right) { //Intake
                IntakeWrist.setPosition(0.90);
            }

            //////////////////////// CLIMB ////////////////////////////////////////

            if (gamepad1.dpad_up && !autoThreadFlag && !armExtension.HorArmFlag) { // go up to top
                autoThreadFlag = true;
                armExtension.HorArmFlag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        climb.setPower(-1);
                        long startTime = System.currentTimeMillis();
                        long duration = 9450; // time in ms for full upward motion
                        armExtension.HorArmFlag = false;
                        autoThreadFlag = false;
                        while (System.currentTimeMillis() - startTime < duration) {
                            telemetry.addData("Distance", climb.getCurrentPosition());
                            telemetry.update();
                            sleep(50);
                        }
                        climb.setPower(0);
                    }
                }).start();
            }

            if (gamepad1.dpad_down && !autoThreadFlag && !armExtension.HorArmFlag) { // down slightly
                autoThreadFlag = true;
                armExtension.HorArmFlag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        climb.setPower(1);
                        long startTime = System.currentTimeMillis();
                        long duration = 700; // time in ms for slight downward motion
                        armExtension.HorArmFlag = false;
                        autoThreadFlag = false;
                        while (System.currentTimeMillis() - startTime < duration) {
                            telemetry.addData("Distance", climb.getCurrentPosition());
                            telemetry.update();
                            sleep(50);
                        }
                        climb.setPower(0);
                    }
                }).start();
            }

            if (gamepad1.dpad_right && !autoThreadFlag && !armExtension.HorArmFlag) { // go all the way down
                autoThreadFlag = true;
                armExtension.HorArmFlag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        climb.setPower(1);
                        long startTime = System.currentTimeMillis();
                        long duration = 9500; // time in ms for full downward motion
                        armExtension.HorArmFlag = false;
                        autoThreadFlag = false;
                        while (System.currentTimeMillis() - startTime < duration) {
                            telemetry.addData("Distance", climb.getCurrentPosition());
                            telemetry.update();
                            sleep(50);
                        }
                        climb.setPower(0);
                    }
                }).start();
            }

            // BUTTON FUNCTION
            //----------------------------------------------------------------

//                if (gamepad2.b && !autoThreadFlag && !armExtension.HorArmFlag) {
//                    autoThreadFlag = true;
//                    armExtension.HorArmFlag = true;
//                    new Thread(new Runnable() {
//                        @Override
//                        public void run() {
//                            armExtension.Arm_Vertical_Position(-0.55, 0.7);
//                            armExtension.HorArmFlag = false;
//                            autoThreadFlag = false;
//                            while (opModeIsActive() && Math.abs(armExtension.getCurrentVerticalLength() - (-0.55)) > 0.01) {
//                                sleep(10);
//                            }
//                            OuttakeWrist.setPosition(0.5); // outtake
//                        }
//                    }).start();
//                }


//            if (gamepad2.a && !autoThreadFlag && !armExtension.HorArmFlag) {
//                autoThreadFlag = true;
//                armExtension.HorArmFlag = true;
//                new Thread(new Runnable() {
//                    @Override
//                    public void run() {
//                        ClawL.setPower(-1);
//                        ClawR.setPower(0.9);
//                        IntakeWrist.setPosition(0.90);
//                    }
//                }).start();
//            }

            // Claw toggle on `x`
            if (gamepad2.x) {
                ClawL.setPower(0); // Stop Power
                ClawR.setPower(0);
            }

            // Reset everything on `y`
            if (gamepad2.y) {
                IntakeWrist.setPosition(0.52); // Neutral position
                OuttakeWrist.setPosition(0);//intake
                ClawL.setPower(0);
                ClawR.setPower(0);
            }

            //----------------------------------------------------------------

            // TRIGGER FUNCTION

            if (gamepad2.left_trigger > 0.01) {//open
                specimen.setPosition(0.3);
            }
            if (gamepad2.right_trigger > 0.01) {//close
                specimen.setPosition(1);
            }
//bucket only
            if (gamepad1.a) {
                OuttakeWrist.setPosition(0.5);//outtake
            }
            if (gamepad1.b) {
                OuttakeWrist.setPosition(0);//intake
            }

            if (gamepad2.a) {
                OuttakeWrist.setPosition(0.5);//outtake
            }
            if (gamepad2.b) {
                OuttakeWrist.setPosition(0);//intake
            }

            if (autoThreadFlag) {
                telemetry.addLine("thread start");

            } else {
                telemetry.addLine("thread end");
            }
            telemetry.addData("Horizontal Slide Length (m)", armExtension.getCurrentHorizontalLength());
            telemetry.addData("Vertical Slide Length (m)", armExtension.getCurrentVerticalLength());
//            telemetry.addData("Red:", colorSensor.red());
//            telemetry.addData("Green:", colorSensor.green());
//            telemetry.addData("Blue:", colorSensor.blue());
            telemetry.addData("Position:", position);

            telemetry.addData("Hue", hsvValues[0]);


            // Add the detected color to telemetry
//            telemetry.addData("Color Detected:", detectedColor);
            // Display the detected color on telemetry
            telemetry.update();
            sleep(33);
        }

    }
}