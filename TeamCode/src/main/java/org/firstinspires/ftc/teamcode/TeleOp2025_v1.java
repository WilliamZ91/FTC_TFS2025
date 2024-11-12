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


    boolean climbBool = false;
    IMU imu;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private boolean autoThreadFlag = false;
    private boolean traverseMode = false;



    private int bButton_DelayCnt = 0;
    private ArmExtension armExtension;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

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

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;



        IntakeWrist.setPosition(0.8);
        sleep(1000);
        OuttakeWrist.setPosition(0.41);

        waitForStart();


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
                speedFactor = 0.5;
            }

            // Control horizontal slide (laterator) with right joystick (horizontal)
            armExtension.controlHorizontalExtend(gamepad2.right_stick_x);

            // Control vertical slide with left joystick (vertical)
            armExtension.controlVerticalExtend(gamepad2.left_stick_y);

            // Add telemetry to show the length of both slides


            // now the orientation of robot is changed
            double forwardBackward = gamepad1.left_stick_y * speedFactor; // Controls forward/backward movement
            double strafe = gamepad1.left_stick_x * speedFactor;          // Controls left/right strafing
            double rotation = gamepad1.right_stick_x * speedFactor;       // Controls rotation

            double denominator = Math.max(Math.abs(forwardBackward) + Math.abs(strafe) + Math.abs(rotation), 1);

            double lfPower = (forwardBackward + strafe + rotation) / denominator;
            double lrPower = (forwardBackward - strafe + rotation) / denominator;
            double rfPower = (forwardBackward - strafe - rotation) / denominator;
            double rrPower = (forwardBackward + strafe - rotation) / denominator;

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


            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            String detectedColor;
            if (hsvValues[0] >= 10 && hsvValues[0] <= 30) {
                detectedColor = "Red";
            } else if (hsvValues[0] >= 70 && hsvValues[0] <= 90) {
                detectedColor = "Yellow";
            } else if (hsvValues[0] >= 200 && hsvValues[0] <= 225) {
                detectedColor = "Blue";
            } else {
                detectedColor = "Unknown";
            }



            if (detectedColor.equals("Red") || detectedColor.equals("Yellow") || detectedColor.equals("Blue")) {
                ClawL.setPower(0);
                ClawR.setPower(0);
            }


            if (gamepad2.dpad_up) {//outtake
                ClawL.setPower(1);
                ClawR.setPower(-1);
            }
            if (gamepad2.dpad_down) {//intake
                ClawL.setPower(-1);
                ClawR.setPower(1);
            }
            if (gamepad2.dpad_right) {//stop
                ClawL.setPower(0);
                ClawR.setPower(0);
            }
            if (gamepad2.a) {
                OuttakeWrist.setPosition(-1);//outtake
            }
            if (gamepad2.b) {//intake
                OuttakeWrist.setPosition(0.47);
            }
            if (gamepad2.left_bumper) {//outtake
                IntakeWrist.setPosition(0.15);
            }
            if (gamepad2.right_bumper) {//intake
                IntakeWrist.setPosition(0.90);
            }
            if (gamepad2.dpad_left) {//traverse
                IntakeWrist.setPosition(0.50);
            }


            /*
            SEMI AUTONOMOUS PLANNING

            Original Position:
            - Traverse Intake
            - Laterator + Vertical Retracted
            - Bucket


            Semi-Auton:
            - Laterator Out
            - Intake Position
            - Intake Wheels
            - Bucket Transfer Position

            - Outtake Position
            - Outtake Wheels
            - Laterator retract

            - Bucket Outtake
            - Vertical Up


         if(gamepad2. insertbutton){
         Arm_HorizontalMotor.RUN_TOPOSITION
         }
             */



            if (autoThreadFlag) {
                telemetry.addLine("thread start");

            } else {
                telemetry.addLine("thread end");
            }
            //telemetry.addData("Horizontal Slide", ArmExtendHardware.Arm_ExtendMotor.getCurrentPosition());
            telemetry.addData("Horizontal Slide Length (m)", armExtension.getCurrentHorizontalLength());
            telemetry.addData("Vertical Slide Length (m)", armExtension.getCurrentVerticalLength());
            telemetry.addData("Red:", colorSensor.red());
            telemetry.addData("Green:", colorSensor.green());
            telemetry.addData("Blue:", colorSensor.blue());


            telemetry.addData("Hue", hsvValues[0]);



            // Add the detected color to telemetry
            telemetry.addData("Color Detected:", detectedColor);
            // Display the detected color on telemetry
            telemetry.update();
            sleep(33);
        }

    }
}