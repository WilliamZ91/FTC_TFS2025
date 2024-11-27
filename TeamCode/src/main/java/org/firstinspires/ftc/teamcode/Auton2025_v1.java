package org.firstinspires.ftc.teamcode;
import android.graphics.SweepGradient;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
@Autonomous(name="TFS_Auton_2025", group="Auto")

public class Auton2025_v1 extends LinearOpMode {
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
    private DcMotorEx climb;
    private boolean autoThreadFlag = false;
    private boolean traverseMode = false;
    private DcMotor Arm_VerticalMotor;   // Vertical slide

    public boolean isLeft = false;
    public boolean isBlue = false;
    public static int blueVal = 1;

    private String curAlliance = "Red";

    boolean togglePreview = true;

    private int bButton_DelayCnt = 0;
    private ArmExtension armExtension;
    double position = 0;

    public void runOpMode() {
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
        specimen.setPosition(1);
        OuttakeWrist.setPosition(0.5);
        sleep(500);
        IntakeWrist.setPosition(0.5);
        while (!opModeIsActive() && !isStopRequested()) {

            waitForStart();


            if (isStopRequested()) return;
            if (!armExtension.VerArmFlag) {
                leftFront.setPower(-.5);
                rightFront.setPower(-.5);
                leftRear.setPower(-.5);
                rightRear.setPower(-.5);
                sleep(70);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(-.5);
                rightFront.setPower(.5);
                leftRear.setPower(.5);
                rightRear.setPower(-.5);
                sleep(1200);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                sleep(50);
                armExtension.Arm_Vertical_Position(-0.41, 1);
                sleep(1000);
                leftFront.setPower(-.5);
                rightFront.setPower(-.5);
                leftRear.setPower(-.5);
                rightRear.setPower(-.5);
                sleep(3000);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                sleep(2000);

                armExtension.Arm_Vertical_Position(-0.30,1);
                while(opModeIsActive() && Math.abs(armExtension.getCurrentVerticalLength() - (-0.30)) > 0.01){
                     sleep(10);
                }
                specimen.setPosition(0.50);
                sleep(1000);
                armExtension.Arm_Vertical_Position(0, 1);
                //test code
                leftFront.setPower(.5);
                rightFront.setPower(.5);
                leftRear.setPower(.5);
                rightRear.setPower(.5);
                sleep(200);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                sleep(2000);
                leftFront.setPower(.5);
                rightFront.setPower(.5);
                leftRear.setPower(.5);
                rightRear.setPower(.5);
                sleep(700);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                sleep(1000);
                leftFront.setPower(-.5);
                rightFront.setPower(.5);
                leftRear.setPower(.5);
                rightRear.setPower(-.5);
                sleep(1490);
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                sleep(200);


                armExtension.VerArmFlag = true;
            }
            while (opModeIsActive()) {

            }
        }
    }
}