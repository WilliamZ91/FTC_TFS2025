package org.firstinspires.ftc.teamcode;

import android.graphics.SweepGradient;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    int armMin = 0;
    int armMax = 5000;
    //double targetAngle = Math.toRadians(90 * TFS_Auton_2024.blueVal);
    // ArmSystem armSystem;
    private DcMotorEx Laterator = null;
    private Servo IntakeWrist = null;
    private Servo OuttakeWrist = null;


    private CRServo ClawL = null;
    private CRServo ClawR = null;

    private double speedfactor = 1.0;
    private double imuAngle = 0.0;


    boolean climbBool = false;
    IMU imu;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private boolean autoThreadFlag = false;
    private boolean traverseMode = false;
    ArmSystem armSystem;



    private int bButton_DelayCnt = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //Need to put armRight and armLeft in hardware map, for armsystem code
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive.setPoseEstimate(PoseStorage.currentPose);
        //DcMotor armLeftMotor = hardwareMap.get(DcMotor.class, "armLeft");
        Laterator = hardwareMap.get(DcMotorEx.class, "Laterator");
        OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");
        IntakeWrist = hardwareMap.get(Servo.class, "IntakeWrist");

        ClawL = hardwareMap.get(CRServo.class, "ClawL");
        ClawR = hardwareMap.get(CRServo.class, "ClawR");





        // arm max = -2300 - outtake position
        // arm min = -100 - intake position

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

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
            armSystem.armTeleOp(gamepad2);

            // now the orientation of robot is changed
            double leftStickXPos = gamepad1.left_stick_x * speedFactor;
            double leftStickYPos = gamepad1.left_stick_y * speedFactor;
            double rightStickXPos = -gamepad1.right_stick_x * speedFactor;

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
                OuttakeWrist.setPosition(0);//outtake
            }
            if (gamepad2.b) {//intake
                OuttakeWrist.setPosition(0.7);
            }
            if (gamepad2.left_bumper) {//outtake
                IntakeWrist.setPosition(0.15);
            }
            if (gamepad2.right_bumper) {//intake
                IntakeWrist.setPosition(0.95);
            }




            if (autoThreadFlag) {
                telemetry.addLine("thread start");
            } else {
                telemetry.addLine("thread end");
            }
            telemetry.update();
            sleep(33);
        }

    }
}