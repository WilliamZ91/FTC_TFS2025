package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmExtension {
    private DcMotor Arm_HorizontalMotor; // Horizontal slide (laterator)
    private DcMotor Arm_VerticalMotor;   // Vertical slide

    // Limits and speed for horizontal (laterator)
    private final double maxHorizontalArm = 0.50; // Max horizontal extension length (meters)
    private final double minHorizontalArm = -0.01; // Min horizontal extension length (meters)
    private final double horizontal_distanceRatio = 2952 / 0.88; // Encoder counts per meter
    private final double speedHorizontal = 0.5; // Speed factor for horizontal slide

    // Limits and speed for vertical slide
    private final double maxVerticalArm = 1; // Max vertical extension length (meters)
    private final double minVerticalArm = -30; // Min vertical extension length (meters)
    private final double vertical_distanceRatio = 4000 / 1.0; // Encoder counts per meter for vertical
    private final double speedVertical = 0.5; // Speed factor for vertical slide

    // Initialize hardware for both slides
    public void initArmExtensionHardware(HardwareMap hardwareMap) {
        // Horizontal slide (laterator)
        Arm_HorizontalMotor = hardwareMap.get(DcMotor.class, "laterator");
        Arm_HorizontalMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust direction if necessary
        Arm_HorizontalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetHorizontalEncoder();

        // Vertical slide
        Arm_VerticalMotor = hardwareMap.get(DcMotor.class, "armL");
        Arm_VerticalMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust direction if necessary
        Arm_VerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetVerticalEncoder();
    }

    // Reset encoder for horizontal slide
    public void resetHorizontalEncoder() {
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Reset encoder for vertical slide
    public void resetVerticalEncoder() {
        Arm_VerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Control horizontal slide (laterator) based on joystick input
    public void controlHorizontalExtend(double gamepadInput) {
        double currentHorizontalPosition = getCurrentHorizontalLength(); // Get current position in meters

        if (gamepadInput < -0.1 && currentHorizontalPosition > minHorizontalArm) { // Retract
            Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_HorizontalMotor.setPower(gamepadInput * speedHorizontal);
        } else if (gamepadInput > 0.1 && currentHorizontalPosition < maxHorizontalArm) { // Extend
            Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_HorizontalMotor.setPower(gamepadInput * speedHorizontal);
        } else {
            Arm_HorizontalMotor.setPower(0); // Stop motor if no input or limits reached
        }
    }

    // Control vertical slide based on joystick input
    public void controlVerticalExtend(double gamepadInput) {
        double currentVerticalPosition = getCurrentVerticalLength(); // Get current position in meters

        if (gamepadInput < -0.1 && currentVerticalPosition > minVerticalArm) { // Retract
            Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_VerticalMotor.setPower(gamepadInput * speedVertical);
        } else if (gamepadInput > 0.1 && currentVerticalPosition < maxVerticalArm) { // Extend
            Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm_VerticalMotor.setPower(gamepadInput * speedVertical);
        } else {
            Arm_VerticalMotor.setPower(0); // Stop motor if no input or limits reached
        }
    }

    // Get the current horizontal slide length in meters
    public double getCurrentHorizontalLength() {
        return Arm_HorizontalMotor.getCurrentPosition() / horizontal_distanceRatio;
    }

    // Get the current vertical slide length in meters
    public double getCurrentVerticalLength() {
        return Arm_VerticalMotor.getCurrentPosition() / vertical_distanceRatio;
    }

    // Helper function to move the horizontal slide to a specific position
    public void Arm_Horizontal_Position(double target, double power) {
        Arm_HorizontalMotor.setTargetPosition((int) (target * horizontal_distanceRatio));
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_HorizontalMotor.setPower(power);
    }

    // Helper function to move the vertical slide to a specific position
    public void Arm_Vertical_Position(double target, double power) {
        Arm_VerticalMotor.setTargetPosition((int) (target * vertical_distanceRatio));
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_VerticalMotor.setPower(power);
    }
}