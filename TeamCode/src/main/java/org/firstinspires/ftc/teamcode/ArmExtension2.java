package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmExtension2 {
    private DcMotor Arm_HorizontalMotor; // Horizontal slide (laterator)
    private DcMotor Arm_VerticalMotor; // Vertical slide

    // Limits and speed for horizontal (laterator)
    private final double maxHorizontalArm = 0.50; // Max horizontal extension length
    private final double minHorizontalArm = 0.01; // Min horizontal extension length
    private final double horizontal_distanceRatio = 2952 / 0.88; // Encoder counts per meter
    private final double speedHorizontal = 0.2; // Speed for horizontal slide

    // Limits and speed for vertical slide
    private final double maxVerticalArm = 0.70; // Max vertical extension length
    private final double minVerticalArm = 0.01; // Min vertical extension length
    private final double vertical_distanceRatio = 6000 / 1.0; // Encoder counts per meter for vertical
    private final double speedVertical = 0.2; // Speed for vertical slide

    // Constructor for initializing the hardware
    public void initArmExtensionHardware(HardwareMap hardwareMap) {
        // Horizontal slide (laterator)
        Arm_HorizontalMotor = hardwareMap.get(DcMotor.class, "laterator");
        Arm_HorizontalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm_HorizontalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetHorizontalEncoder();

        // Vertical slide
        Arm_VerticalMotor = hardwareMap.get(DcMotor.class, "armL"); // Vertical slide hardware
        Arm_VerticalMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust direction if needed
        Arm_VerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetVerticalEncoder();
    }

    // Reset encoder functions
    public void resetHorizontalEncoder() {
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetVerticalEncoder() {
        Arm_VerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Function to control horizontal slide (laterator)
    public void controlHorizontalExtend(double gamepadInput) {
        double currentHorizontalPosition = Arm_HorizontalMotor.getCurrentPosition() / horizontal_distanceRatio;

        if (gamepadInput < -0.1 && currentHorizontalPosition > minHorizontalArm) { // Retract
            //Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double targetHorizontalArm = currentHorizontalPosition + (gamepadInput * speedHorizontal);
            if (targetHorizontalArm < minHorizontalArm) targetHorizontalArm = minHorizontalArm;
            Arm_Horizontal_Position(targetHorizontalArm, 0.75);
        } else if (gamepadInput > 0.1 && currentHorizontalPosition < maxHorizontalArm) { // Extend
            //Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double targetHorizontalArm = currentHorizontalPosition + (gamepadInput * speedHorizontal);
            if (targetHorizontalArm > maxHorizontalArm) targetHorizontalArm = maxHorizontalArm;
            Arm_Horizontal_Position(targetHorizontalArm, 0.75);
        }

        /*if (currentHorizontalPosition >= maxHorizontalArm || currentHorizontalPosition <= minHorizontalArm) {
           // resetHorizontalEncoder();
        }*/
    }

    // Function to control vertical slide
    public void controlVerticalExtend(double gamepadInput) {
        double currentVerticalPosition = Arm_VerticalMotor.getCurrentPosition() / vertical_distanceRatio;

        if (gamepadInput < -0.1 && currentVerticalPosition > minVerticalArm) { // Retract
//            Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double targetVerticalArm = currentVerticalPosition + (gamepadInput * speedVertical);
            if (targetVerticalArm < minVerticalArm) targetVerticalArm = minVerticalArm;
            Arm_Vertical_Position(targetVerticalArm, 0.75);
        } else if (gamepadInput > 0.1 && currentVerticalPosition < maxVerticalArm) { // Extend
//            Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double targetVerticalArm = currentVerticalPosition + (gamepadInput * speedVertical);
            if (targetVerticalArm > maxVerticalArm) targetVerticalArm = maxVerticalArm;
            Arm_Vertical_Position(targetVerticalArm, 0.75);
        }

        /*if (currentVerticalPosition >= maxVerticalArm || currentVerticalPosition <= minVerticalArm) {
            //resetVerticalEncoder();
        }*/
    }

    // Helper functions for positioning
    public void Arm_Horizontal_Position(double target, double power) {
        Arm_HorizontalMotor.setTargetPosition((int) (target * horizontal_distanceRatio));
        Arm_HorizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_HorizontalMotor.setPower(power);
    }

    public void Arm_Vertical_Position(double target, double power) {
        Arm_VerticalMotor.setTargetPosition((int) (target * vertical_distanceRatio));
        Arm_VerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm_VerticalMotor.setPower(power);
    }

    // Functions for telemetry feedback on current positions
    public double getCurrentHorizontalLength() {
        return Arm_HorizontalMotor.getCurrentPosition() / horizontal_distanceRatio;
    }

    public double getCurrentVerticalLength() {
        return Arm_VerticalMotor.getCurrentPosition() / vertical_distanceRatio;
    }
}