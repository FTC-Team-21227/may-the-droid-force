package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "grace setup tele :)")
public class TestTeleop082425 extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();
    TelemetryPacket packet = new TelemetryPacket();

    DcMotor backLeftWheel;
    DcMotor backRightWheel;
    DcMotor frontRightWheel;
    DcMotor frontLeftWheel;

    DcMotor lift; // Closest to back, Port 0
    DcMotor lift_S1; // Middle, Port 1
    DcMotor lift_S2; // Closest to the front, Port 2

    Servo ext_Left; // In: 0.4, out: 0.7
    Servo ext_Right; // In: 0.66, out: 0.85

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
    public static double leftTargetExt, rightTargetExt;
    public static float leftExtLow, leftExtHigh, rightExtLow, rightExtHigh;

    double motorPowerFL;
    double motorPowerBL;
    double motorPowerBR;
    double motorPowerFR;

    @Override
    public void runOpMode() throws InterruptedException {

        liftMotorPower = 0.5;
        leftExtLow = 0; leftExtHigh = 1; rightExtLow = 0; rightExtHigh = 1;

        initialization();

        // Servo positions
        ext_Right.scaleRange(rightExtLow, rightExtHigh);
        ext_Left.scaleRange(leftExtLow, leftExtHigh);

        leftTargetExt = 0;
        rightTargetExt = 0;

        if (gamepad1.right_trigger > 0.1) {
            if (gamepad1.triangleWasPressed()) {
                rightTargetExt += 0.1;
            }
            if (gamepad1.crossWasPressed()) {
                rightTargetExt -= 0.1;
            }
        }

        if (gamepad1.left_trigger > 0.1) {
            if (gamepad1.triangleWasPressed()) {
                leftTargetExt += 0.1;
            }
            if (gamepad1.crossWasPressed()) {
                leftTargetExt -= 0.1;
            }
        }


        waitForStart();
        while(opModeIsActive()) {

//            calculateIMURotationPower();
//            calculateMotorPower();
//            setMotorPower();



            // Stop the lifts
            if(gamepad1.leftBumperWasPressed() && gamepad1.rightBumperWasPressed()) {
                lift.setPower(0);
                lift_S1.setPower(0);
            }


            // Adjust lift motor power using dpad up and down arrows
            if (gamepad1.dpadUpWasPressed()) {
                liftMotorPower += 0.1;
            }
            if (gamepad1.dpadDownWasPressed()) {
                liftMotorPower -= 0.1;
            }

            // Lift motor powers based on gamepad1 right stick
            lift.setPower(-gamepad1.right_stick_y * liftMotorPower);
            lift_S1.setPower(-gamepad1.left_stick_y * liftMotorPower);

            ext_Right.setPosition(rightTargetExt);
            ext_Left.setPosition(leftTargetExt);

            dashTelemetry.addData("left servo target pos", leftTargetExt);
            dashTelemetry.addData("right servo target pos", rightTargetExt);
            dashTelemetry.addLine();
            dashTelemetry.addData("left servo pos", ext_Left.getPosition());
            dashTelemetry.addData("right servo pos", ext_Right.getPosition());
            dashTelemetry.addLine();
            dashTelemetry.addData("lift power factor", liftMotorPower);
            dashTelemetry.addData("lift pos", lift.getCurrentPosition());
            dashTelemetry.addData("lift power", lift.getPower());
            dashTelemetry.addData("lift_1 pos", lift_S1.getCurrentPosition());
            dashTelemetry.addData("lift_1 power", lift_S1.getPower());
            dashTelemetry.update();
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
        lift_S1.setDirection(DcMotorSimple.Direction.REVERSE);

        ext_Left.setDirection(Servo.Direction.REVERSE);


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
