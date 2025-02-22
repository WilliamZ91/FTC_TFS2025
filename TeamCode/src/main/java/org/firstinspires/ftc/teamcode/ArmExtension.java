package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmExtension {
    private DcMotor Arm_HorizontalMotor; // Horizontal slide (laterator)
    private DcMotor Arm_VerticalMotor;   // Vertical slide

    // Limits and speed for horizontal slide
    private final double maxHorizontalArm = 1;         // Maximum horizontal extension (meters)
    private final double minHorizontalArm = -30;       // Minimum horizontal extension (meters)
    private final double horizontal_distanceRatio = 4000.0; // Encoder counts per meter
    private final double speedHorizontal = 0.6;

    // Limits and speed for vertical slide
    private final double maxVerticalArm = 1;           // Maximum vertical extension (meters)
    private final double minVerticalArm = -30;         // Minimum vertical extension (meters)
    private final double vertical_distanceRatio = 4000.0;   // Encoder counts per meter
    private final double speedVertical = 0.8;

    public boolean HorArmFlag;
    public boolean VerArmFlag;

    // Reference to opMode for sleep and active checking.
    private LinearOpMode opMode;

    // Pass the current op mode to this class so we can call opMode.sleep and opModeIsActive().
    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    // Initialize hardware for both slides.
    public void initArmExtensionHardware(HardwareMap hardwareMap) {
        // Horizontal slide (laterator)
        Arm_HorizontalMotor = hardwareMap.get(DcMotor.class, "laterator");
        Arm_HorizontalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm_HorizontalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetHorizontalEncoder();

        // Vertical slide
        Arm_VerticalMotor = hardwareMap.get(DcMotor.class, "armL");
        Arm_VerticalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm_VerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetVerticalEncoder();
    }

    // Reset encoder for horizontal slide.
    public void resetHorizontalEncoder() {
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Reset encoder for vertical slide.
    public void resetVerticalEncoder() {
        Arm_VerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Teleop control for horizontal slide.
    public void controlHorizontalExtend(double gamepadInput) {
        double currentHorizontalPosition = getCurrentHorizontalLength();
        if (gamepadInput < -0.1 && currentHorizontalPosition > minHorizontalArm) { // Retract
            Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_HorizontalMotor.setPower(gamepadInput * speedHorizontal);
        } else if (gamepadInput > 0.1 && currentHorizontalPosition < maxHorizontalArm) { // Extend
            Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_HorizontalMotor.setPower(gamepadInput * speedHorizontal);
        } else if (!HorArmFlag) {
            Arm_HorizontalMotor.setPower(0);
        }
    }

    // Teleop control for vertical slide.
    public void controlVerticalExtend(double gamepadInput) {
        double currentVerticalPosition = getCurrentVerticalLength();
        if (gamepadInput < -0.1 && currentVerticalPosition > minVerticalArm) { // Retract
            Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_VerticalMotor.setPower(gamepadInput * speedVertical);
        } else if (gamepadInput > 0.1 && currentVerticalPosition < maxVerticalArm) { // Extend
            Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_VerticalMotor.setPower(gamepadInput * speedVertical / 5);
        } else if (!VerArmFlag) {
            Arm_VerticalMotor.setPower(0);
        }
    }

    // Get current horizontal slide length in meters.
    public double getCurrentHorizontalLength() {
        return Arm_HorizontalMotor.getCurrentPosition() / horizontal_distanceRatio;
    }

    // Get current vertical slide length in meters.
    public double getCurrentVerticalLength() {
        return Arm_VerticalMotor.getCurrentPosition() / vertical_distanceRatio;
    }

    // Move horizontal slide to a target position.
    public void Arm_Horizontal_Position(double target, double power) {
        int pos = (int)(target * horizontal_distanceRatio);
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_HorizontalMotor.setTargetPosition(pos);
        if (opMode != null) {
            opMode.sleep(50);
            if (!opMode.opModeIsActive()) return;
        }
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_HorizontalMotor.setPower(power);
    }

    // Move vertical slide to a target position.
    public void Arm_Vertical_Position(double target, double power) {
        int pos = (int)(target * vertical_distanceRatio);
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_VerticalMotor.setTargetPosition(pos);
        if (opMode != null) {
            opMode.sleep(50);
            if (!opMode.opModeIsActive()) return;
        }
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_VerticalMotor.setPower(power);
    }
}
