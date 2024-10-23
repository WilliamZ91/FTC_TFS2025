package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class FTCSim_Test extends LinearOpMode {
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor armTilt;
    DcMotor armExtend;
    DcMotor claw;
    IMU imu;

    @Override
    public void runOpMode() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "lr");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        imu = hardwareMap.get(IMU.class, "imu");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
       backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//for the big robot chassis


        // Put initialization blocks here
        waitForStart();
        // Put run blocks here

//        backLeftDrive.setPower(1);
//        backRightDrive.setPower(1);
//        frontLeftDrive.setPower(1);
//        frontRightDrive.setPower(1);
//        sleep(150);

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                backLeftDrive.setPower(0.5);
            }
            if (gamepad1.b) {
                frontLeftDrive.setPower(0.5);
            }
            if (gamepad1.x) {
                backRightDrive.setPower(0.5);
            }
            if (gamepad1.y) {
                frontRightDrive.setPower(0.5);
            }
            if (gamepad1.dpad_down) {
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                frontRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
            }
        }
    }

}
