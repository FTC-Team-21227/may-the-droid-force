package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class tele extends LinearOpMode {
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
        while(opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;
            double leftFront = x+y+r;
            double rightFront = x-y-r;
            double leftBack = x-y+r;
            double rightBack = x+y-r;
            W_FL.setPower(leftFront);
            W_FR.setPower(rightFront);
            W_BL.setPower(leftBack);
            W_BR.setPower(rightBack);

        }
    }
    public void initialize(){
        W_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        W_BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
