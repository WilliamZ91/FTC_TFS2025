package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="FTC SIM", group="Linear OpMode")
public class FTCSim_Test extends LinearOpMode {
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor armTilt;
    DcMotor armExtend;
    DcMotor claw;
    BNO055IMU imu;

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "lr");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        // Put initialization blocks here
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        // Put run blocks here
        backLeftDrive.setPower(1);
        backRightDrive.setPower(1);
        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        sleep(100); 
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        sleep(1000);
        while (opModeIsActive()) {
            // Put loop blocks here
        }
    }
}
