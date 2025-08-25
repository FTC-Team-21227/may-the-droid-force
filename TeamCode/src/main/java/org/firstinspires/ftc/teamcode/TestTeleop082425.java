package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "grace teleop 1")
public class TestTeleop082425 extends LinearOpMode {
    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor frontRightWheel;
    DcMotor frontLeftWheel;

    DcMotor lift;
    DcMotor lift_S1;
    DcMotor lift_S2;

    Servo ext_Left;
    Servo ext_Right;

    IMU imu;

    Orientation direction;
    int imuRotation;
    double targetingAngle = 0;
    double headingAngle = 0;
    double initialHeading = 0;
    double motorFwdPower;
    double motorSidePower;
    double motorRotationPower;
    double motorPower = 0.6;
    double angleDifference;

    int liftTargetPos = 0;
    double liftMotorPower = 1;
    double target_Ext;

    double motorPowerFL;
    double motorPowerBL;
    double motorPowerBR;
    double motorPowerFR;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while(opModeIsActive()) {
            calculateIMURotationPower();
            calculateMotorPower();
            setMotorPower();

            if (gamepad1.triangle) {
                lift.setTargetPosition(500);
                liftMotorPower = lift.getTargetPosition() - lift.getCurrentPosition();
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(liftMotorPower);
            }
            if (gamepad1.cross) {
                lift.setTargetPosition(0);
                liftMotorPower = lift.getTargetPosition() - lift.getCurrentPosition();
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(liftMotorPower);
            }
            if (gamepad1.left_bumper) {
                if (gamepad1.square) {
                    ext_Left.setPosition(0.4);
                }
                if (gamepad1.circle) {
                    ext_Right.setPosition(0.4);
                }
                if(gamepad1.triangle) {
                    ext_Left.setPosition(0.5);
                }
                if(gamepad1.cross) {
                    ext_Left.setPosition(0.5);
                }
            }

            telemetry.update();
        }
    }

    /**
     * Initialize all hardware with correct settings.
     */
    public void initialization() {
        backLeftWheel = hardwareMap.get(DcMotor.class, "W_BL");
        backRightWheel = hardwareMap.get(DcMotor.class, "W_BR");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "W_FL");
        frontRightWheel = hardwareMap.get(DcMotor.class, "W_FR");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift_S1 = hardwareMap.get(DcMotor.class, "lift_S1");
        lift_S2 = hardwareMap.get(DcMotor.class, "lift_S2");
        ext_Left = hardwareMap.get(Servo.class, "ext_Left");
        ext_Right = hardwareMap.get(Servo.class, "ext_Right");
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw(); // Reset IMU heading
    }

    /**
     * Calculate rotation power based on IMU heading reading, stored in imuRotation.
     */
    private void calculateIMURotationPower() {
        direction = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingAngle = direction.firstAngle;
        if (Math.abs(gamepad1.right_stick_x) >= 0.01) { // Target and heading angle close enough
            imuRotation = 0;
            targetingAngle = headingAngle;
        } else {
            angleDifference = headingAngle - targetingAngle;
            if (angleDifference > 180) { // Prevent jerky full-circle turns
                angleDifference = angleDifference - 360;
            } else if (angleDifference < -180) {
                angleDifference = angleDifference + 360;
            }
            if (Math.abs(angleDifference) < 1) {
                imuRotation = 0;
            } else if (angleDifference >= 1) {
                imuRotation = (int) (angleDifference * 0);
            } else {
                imuRotation = (int) (angleDifference * 0);
            }
        }
    }

    /**
     * Calculate individual motor powers
     */
    private void calculateMotorPower() {
        float motorFwdInput;
        float motorSideInput;
        motorFwdInput = -gamepad1.left_stick_y;
        motorSideInput = gamepad1.left_stick_x;
        motorFwdPower = Math.cos(headingAngle / 180 * Math.PI) * motorFwdInput - Math.sin(headingAngle / 180 * Math.PI) * motorSideInput;
        motorSidePower = (Math.cos(headingAngle / 180 * Math.PI) * motorSideInput + Math.sin(headingAngle / 180 * Math.PI) * motorFwdInput) * 1.5;
        motorRotationPower = gamepad1.right_stick_x * 0.7 + imuRotation;
        motorPowerBL = (motorFwdPower - motorSidePower + motorRotationPower) * motorPower;
        motorPowerBR = (motorFwdPower + motorSidePower - motorRotationPower) * motorPower;
        motorPowerFL = (motorFwdPower + motorSidePower + motorRotationPower * motorPower);
        motorPowerFR = (motorFwdPower - motorSidePower - motorRotationPower) * motorPower;
    }

    /**
     * Set motor powers
     */
    private void setMotorPower() {
        backLeftWheel.setPower(motorPowerBL);
        backRightWheel.setPower(motorPowerBR);
        frontLeftWheel.setPower(motorPowerFL);
        frontRightWheel.setPower(motorPowerFR);
    }
}
