package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmExtendHardware {
    private DcMotor Arm_ExtendMotor;

    // Arm extension limits and speed
    private final double maxExtendArm = 0.48; // Maximum extension length (adjust this based on your setup)
    private final double minExtendArm = 0.01; // Minimum extension length
    private final double Arm_extend_distanceRatio = 2952 / 0.88; // Encoder counts per meter
    private final double speedExtend = 0.2; // Increased speed for smoother and faster extension

    // Constructor for initializing the hardware
    public void initArmExtendHardware(HardwareMap hardwareMap) {
        Arm_ExtendMotor = hardwareMap.get(DcMotor.class, "laterator");
        Arm_ExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust direction if needed
        Arm_ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder(); // Reset encoder on initialization
    }

    // Reset encoder function
    public void resetEncoder() {
        Arm_ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Function to extend the arm to a target position (in meters)
    public void Arm_Extend_Position(double target, double power) {
        if (target < minExtendArm) target = minExtendArm;
        if (target > maxExtendArm) target = maxExtendArm;

        Arm_ExtendMotor.setTargetPosition((int) (target * Arm_extend_distanceRatio)); // Convert meters to encoder counts
        Arm_ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_ExtendMotor.setPower(power);

        while (Arm_ExtendMotor.isBusy()) {
            // Wait until the arm reaches the target position
        }
        Arm_ExtendMotor.setPower(0); // Stop the motor once the target is reached
    }

    // Tele-op function to control arm extension with gamepad input and provide smooth movement
    public void controlArmExtend(double gamepadInput) {
        double currentExtendPosition = Arm_ExtendMotor.getCurrentPosition() / Arm_extend_distanceRatio;

        // Update the logic to allow retracting even if at maximum extension
        if (gamepadInput < -0.1 && currentExtendPosition > minExtendArm) { // Retract
            double targetExtendArm = currentExtendPosition + (gamepadInput * speedExtend);
            if (targetExtendArm < minExtendArm) targetExtendArm = minExtendArm;
            Arm_Extend_Position(targetExtendArm, 0.75); // Increased power for smoother movement
        } else if (gamepadInput > 0.1 && currentExtendPosition < maxExtendArm) { // Extend
            double targetExtendArm = currentExtendPosition + (gamepadInput * speedExtend);
            if (targetExtendArm > maxExtendArm) targetExtendArm = maxExtendArm;
            Arm_Extend_Position(targetExtendArm, 0.75); // Increased power for smoother movement
        }
    }
}
