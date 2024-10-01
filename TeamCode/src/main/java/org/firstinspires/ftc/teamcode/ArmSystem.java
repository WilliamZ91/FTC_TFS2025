package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import javax.lang.model.element.VariableElement;

public class ArmSystem {

    private static final double SLIDER_PPR = 537.7;
    public          double SLIDER_STEP = 0.2;

    private static final double PULL_WHEEL_DIA = 0.04;
    public boolean slow = false;// unit: m
    private static final int SLIDER_STAGE = 4;
    private static final double SLIDER_STAGE_LEN = 0.1746; // unit: m
    private final int SLIDER_MIN = 0;
    private final int SLIDER_MAX= 3000; //(int)(SLIDER_STAGE * SLIDER_STAGE_LEN / (Math.PI * (PULL_WHEEL_DIA * 3)) * SLIDER_PPR); // Max slider position
    private static final double SLIDER_LEN_RES = (Math.PI * PULL_WHEEL_DIA) / SLIDER_PPR;
    private static volatile double sliderBtnCnt = 0;
    private static volatile double sliderPwr = 0;
    private double slideStep = 0.2;
    private static volatile double sliderMinPwr = 0.3;
    // Slider related instance variables
    public DcMotorEx leftArmMtr = null;
    public DcMotorEx rightArmMtr = null;

    private int sliderPosLeft;
    private int sliderPosRight;
    private volatile double sliderRightLen;
    private volatile double sliderLeftLen;
    public boolean armflag;

    public ArmSystem(LinearOpMode mainTask, boolean auto) {
        leftArmMtr = mainTask.hardwareMap.get(DcMotorEx.class, "armL");
        rightArmMtr = mainTask.hardwareMap.get(DcMotorEx.class, "armR");
        leftArmMtr.setDirection(DcMotor.Direction.FORWARD);
        leftArmMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMtr.setDirection(DcMotor.Direction.REVERSE);
        rightArmMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightArmMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftArmMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeftLen = getSlideLen("left");
        sliderRightLen = getSlideLen("right");
        sliderPosRight = rightArmMtr.getCurrentPosition();
        sliderPosLeft = rightArmMtr.getCurrentPosition();
    }
    public void sliderCtrl(String motor, int position, double power) {
        switch (motor) {
            case "left":
                position = Math.min(position, SLIDER_MAX);
                position = Math.max(position, SLIDER_MIN);
                this.sliderPosLeft = position;
                leftArmMtr.setTargetPosition(position);
                leftArmMtr.setPower(power);
                leftArmMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "right":
                position = Math.min(position, SLIDER_MAX);
                position = Math.max(position, SLIDER_MIN);
                this.sliderPosRight = position;
                rightArmMtr.setTargetPosition(position);
                rightArmMtr.setPower(power);
                rightArmMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "both":
                position = Math.min(position, SLIDER_MAX);
                position = Math.max(position, SLIDER_MIN);
                this.sliderPosRight = position;
                this.sliderPosLeft = position;
                leftArmMtr.setTargetPosition(position);
                leftArmMtr.setPower(power);
                leftArmMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArmMtr.setTargetPosition(position);
                rightArmMtr.setPower(power);
                rightArmMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    public void sliderLenCtrl(String motor, double len, double power) {
        int position = (int)(len / SLIDER_LEN_RES);
        position = Math.min(position, SLIDER_MAX);
        position = Math.max(position, SLIDER_MIN);
        sliderCtrl(motor, position, power);
    }
    public void stopRobotArm() {
        rightArmMtr.setPower(0);
        leftArmMtr.setPower(0);
    }
    private void sliderHoldPos(String motor, double power) {
        leftArmMtr.setTargetPosition(leftArmMtr.getCurrentPosition());
        rightArmMtr.setTargetPosition(rightArmMtr.getCurrentPosition());
        leftArmMtr.setPower(power);
        rightArmMtr.setPower(power);
        leftArmMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getSlideLen(String motor) {
        switch (motor) {
            case "left":
                return leftArmMtr.getCurrentPosition() * SLIDER_LEN_RES;
            case "right":
                return rightArmMtr.getCurrentPosition() * SLIDER_LEN_RES;
            default:
                return 0;
        }
    }

    public void slowMoveSliderMtr(double pwr, DcMotorEx motor) {
        motor.setPower(pwr);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void armTeleOp(Gamepad gamePad)
    {
        double slidePower = -gamePad.right_stick_y;
        if (Math.abs(slidePower) > 0.3) {
            sliderLenCtrl("both", getSlideLen("left") + slidePower * SLIDER_STEP, slidePower);
        } else if (!armflag) {
            sliderHoldPos("both", sliderMinPwr);
        }
        if(gamePad.back&&!slow){
            SLIDER_STEP = 0.02;
            slow = true;
        }
        else if(gamePad.back&&slow){
            SLIDER_STEP = 0.2;
            slow = false;
        }
    }


}
