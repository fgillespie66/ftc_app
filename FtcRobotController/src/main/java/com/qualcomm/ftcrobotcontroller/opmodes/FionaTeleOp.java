/*package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by cflee4 on 10/24/15.
 */
/*public class FionaTeleOp extends OpMode {
    //2 ways to write code
    // always import the import things at the top
    //Way 1: extends LinearOpMode
    //uses public void (in the sample code)
    //Way 2: extends OpMode
    //extend imports methods
    //in both Linear and regular, you must declare variables right after you declare class
    //in both declare motors and motor controllers and servos
    // in non-linear, if you want something to happen at the beginning, put it in public [className]. (e.g. public K9TeleOp).
    //using Linear mode

    float motorPowerArm = 0;
    float motorPowerLeft = 0;
    float motorPowerRight = 0;

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

    //this initializes the boolean
    int reverse = 0;
    boolean up = false;
    float wristPosition = 0;
    float sweeperLeft = 0;
    float sweeperRight = 0;
    float temp;

    @Override
    public void init() {
        //this is for mapping hardware (hook up variables to actual devices in robot)
        MotorController1 = hardwareMap.dcMotorController.get("MotorController1");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorArm = hardwareMap.dcMotor.get("motorArm");

        MotorController2 = hardwareMap.dcMotorController.get("MotorController2");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        ServoController1 = hardwareMap.servoController.get("ServoController1");
        leftShovel = hardwareMap.servo.get("leftShovel");
        rightShovel = hardwareMap.servo.get("rightShovel");
        leftWindshieldWiper = hardwareMap.servo.get("leftWindshieldWiper");
        rightWindshieldWiper = hardwareMap.servo.get("rightWindshieldWiper");

        //just in case, set all motors to direction forward
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorArm.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {

        //true = 1, false = 0
        temp = gamepad1.left_trigger;
        //gives a decimal
        temp = Math.round(temp);
        //rounds to 0 or 1
        reverse = (int)temp;
        if (reverse == 1) {
            //this is the special thing
            //left trigger reverses everything
            //test whether x must be continually held or just pressed once
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
            motorRight.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
            motorRight.setDirection(DcMotor.Direction.FORWARD);
        }

        if (gamepad2.x) {
            motorArm.setPower(1);
            motorPowerArm = 1;
        }

        else {
            motorArm.setPower(0);
        }

        motorPowerLeft = gamepad1.left_stick_y;
        motorPowerRight = gamepad1.right_stick_y;
        motorPowerRight = Range.clip(motorPowerRight, -1, 1);
        motorPowerLeft = Range.clip(motorPowerLeft, -1, 1);

        motorPowerRight = (float)scaleInput(motorPowerRight);
        motorPowerLeft = (float)scaleInput(motorPowerLeft);

        motorLeft.setPower(motorPowerLeft);
        motorRight.setPower(motorPowerRight);


        /*gamepad1.left_stick_y = float

            motorLeft.setPower();
            motorPowerLeft = 1;
        }
        if(gamepad1.left_stick_y < 0){
            motorLeft.setPower();
            motorPowerLeft = -1;
        }

        else {
            motorLeft.setPower(0);
        }


        if (gamepad1.right_stick_y != 0) {
            motorRight.setPower(1);
            motorPowerRight = 255;
        }

        else {
            motorRight.setPower(0);
        }

        if (motorPowerRight == 0 && motorPowerLeft ==0) {
            leftShovel.setPosition(0);
            rightShovel.setPosition(0);
            leftWindshieldWiper.setPosition(0);
            rightWindshieldWiper.setPosition(0);

        }

        if (gamepad2.y) {
            wristPosition += 10;
            //metal piece moves in increments of 10
            //set servos to position
            //+ goes up
            leftShovel.setPosition(wristPosition);
            rightShovel.setPosition(wristPosition);
        }
        if (gamepad2.a) {
            wristPosition -= 10;
            //metal piece moves in increments of 10
            //set servos to position
            //- goes down
            leftShovel.setPosition(wristPosition);
            rightShovel.setPosition(wristPosition);
        }


        /*left joystick moves rack and pinion up and down
        on left cross, left button moves leftWindshieldWiper out, right button moves leftWindshieldWiper in
        x moves rightWindshieldWiper in
        b moves rightWindshieldWiper out
        y moves shovel up (specific degrees)
        a moves shovel down
         */
       /* if (gamepad2.dpad_left) {
            //left part of cross
            sweeperLeft += 10;
            leftWindshieldWiper.setPosition(sweeperLeft);
        }
        if (gamepad2.dpad_right) {
            //right part of cross
            sweeperLeft -= 10;
            leftWindshieldWiper.setPosition(sweeperLeft);
        }
        if (gamepad2.x) {
            sweeperRight += 10;
            rightWindshieldWiper.setPosition(sweeperRight);
        }
        if (gamepad2.b) {
            sweeperRight -= 10;
            rightWindshieldWiper.setPosition(sweeperRight);
        }


    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


    public void stop() {
        //code that does stuff when teleop ends

    }


    //main loop of code
    //everytime you input something, it runs through this loop


}*/