package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "hi")
public class teleop extends LinearOpMode {
    DcMotor W_FL;
    DcMotor W_FR;
    DcMotor W_BL;
    DcMotor W_BR;
    public void runOpMode(){
        W_FL = hardwareMap.get(DcMotor.class,"W_FL");
        W_FR = hardwareMap.get(DcMotor.class,"W_FR");
        W_BL = hardwareMap.get(DcMotor.class,"W_BL");
        W_BR = hardwareMap.get(DcMotor.class,"W_BR");
        initialize();
        waitForStart();
        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            double leftFront = x+y+r;
            double rightFront = x-y-r;
            double leftBack = x-y+r;
            double rightBack = x+y-r;
            double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(max, Math.abs(leftBack));
            max = Math.max(max, Math.abs(rightBack));
            if (max > 1){
                leftFront /= max;
                rightFront /= max;
                leftBack /= max;
                rightBack /= max;
            }
            W_FL.setPower(leftFront);
            W_FR.setPower(rightFront);
            W_BL.setPower(leftBack);
            W_BR.setPower(rightBack);
        }
    }
    public void initialize(){
        W_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        W_FR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
