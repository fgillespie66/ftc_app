package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
/**
 * Created by cflee4 on 10/25/15.
 */
public class runMe extends LinearOpMode {
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorArm;

    DcMotorController.DeviceMode devMode;
    DcMotorController MotorController1;
    DcMotorController MotorController2;

    ServoController ServoController1;

    Servo leftShovel;
    Servo rightShovel;
    Servo leftWindshieldWiper;
    Servo rightWindshieldWiper;

    @Override
    public void runOpMode() throws InterruptedException{

        MotorController1 = hardwareMap.dcMotorController.get("MotorController1");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorRight = hardwareMap.dcMotor.get("MotorRight");

        MotorController2 = hardwareMap.dcMotorController.get("MotorController2");
        motorLeft = hardwareMap.dcMotor.get("MotorLeft");

        ServoController1 = hardwareMap.servoController.get("ServoController1");
        leftShovel = hardwareMap.servo.get("leftShovel");
        rightShovel = hardwareMap.servo.get("rightShovel");
        leftWindshieldWiper = hardwareMap.servo.get("leftWindshieldWiper");
        rightWindshieldWiper = hardwareMap.servo.get("rightWindshieldWiper");

        //just in case, set all motors to direction forward
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {
            motorLeft.setPower(1);
            motorRight.setPower(1);
            motorArm.setPower(1);
        }
    }


}


