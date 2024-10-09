package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Arm Extension TeleOp", group = "Linear Opmode")
public class ArmExtendHardware extends LinearOpMode {

    // Declare arm extension hardware component
    private DcMotor Arm_ExtendMotor;

    // Arm extension limits and speed
    private final double maxExtendArm = 0.6; // Maximum extension length (in meters)
    private final double minExtendArm = 0.01; // Minimum extension length
    private final double Arm_extend_distanceRatio = 2952 / 0.88; // Encoder counts per meter
    private final double speedExtend = 0.1; // Speed for extending the arm

    @Override
    public void runOpMode() {
        initArmExtendHardware();


        waitForStart();

        while (opModeIsActive()) {
            controlArmExtend(gamepad2.right_stick_y);
             // Control the arm extension with right joystick input
        }
    }

    // Initialize the hardware component for arm extension
    public void initArmExtendHardware() {
        Arm_ExtendMotor = hardwareMap.dcMotor.get("laterator");
        Arm_ExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust direction if needed
        Arm_ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    // Use this method in your tele-op loop to control the arm extension with gamepad input
    public void controlArmExtend(double gamepadInput) {
        double currentExtendPosition = Arm_ExtendMotor.getCurrentPosition() / Arm_extend_distanceRatio;

        if (Math.abs(gamepadInput) > 0.1) { // Check for significant input from the gamepad
            double targetExtendArm = currentExtendPosition + (gamepadInput * speedExtend);
            Arm_Extend_Position(targetExtendArm, 0.5); // Adjust power based on your need
        }
    }
}
