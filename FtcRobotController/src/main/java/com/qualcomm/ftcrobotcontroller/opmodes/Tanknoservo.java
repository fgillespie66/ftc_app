//package com.qualcomm.ftcrobotcontroller.opmodes;

/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

//package com.qualcomm.ftcrobotcontroller.opmodes;


/*
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoController;
*/

/**
 * TeleOp Code - Continuous Track Drive System (Tank Tread Drive)
 * Team: Positive Charge 10343
 * By Rachael H. and Saranya T.
 * Created on 20 November 2015
 */


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class Tanknoservo extends OpMode {

    //Removed code
    // TETRIX VALUES.
    //Removed code
    float left;
    float right;
    //Removed code

    float up1;
    float down1;
    float left1;
    float right1;

    //Removed code

    float motorPowerArm = 0;
    float motorPowerTreadLeft = 0;
    float motorPowerTreadRight = 0;
    float motorPowerAutonomous = 0;
    float motorPowerPullupLeft = 0;
    float motorPowerPullupRight = 0;
    float motorPowerPivotLeft = 0;
    float motorPowerPivotRight = 0;

    //DcMotorController.DeviceMode devMode;
    DcMotorController motorController1; //Changed identifier
    DcMotor motorTreadRight;
    DcMotor motorTreadLeft;

    DcMotorController motorController2; //Changed identifier
    DcMotor motorArm;
    DcMotor motorAutonomous;

    DcMotorController motorController3; //Changed identifier
    DcMotor motorPullupRight;
    DcMotor motorPullupLeft;

    DcMotorController motorController4; //Changed identifier
    DcMotor motorPivotRight;
    DcMotor motorPivotLeft;

    ServoController servoController1; //Changed identifier
    Servo leftShovel;
    Servo rightShovel;
    Servo leftWindshieldWiper;
    Servo rightWindshieldWiper;
    Servo shovelVertical1;
    Servo shovelVertical2;

    ServoController servoController2; //Changed identifier
    Servo sideFlap; //Switched to other servo controller

    //LegacyModule legacyModule;

    //Initializes boolean
    int reverse = 0;
    boolean up = false;
    //Removed code
    float sweeperLeft = 0;
    // float sweeperRight = 0;
    float temp;

    // boolean sideFlapB = false; //Boolean for side flap
    float sideFlapPosition = 0;

    double shovelV1Position = 0;
    //Left servo
    double shovelV2Position = 0;
    //Right servo

    //Removed code

	/*
	 * Constructor
	 */

    public Tanknoservo() {

    }

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */

    @Override
    public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        //Removed code

        //in thq
        motorTreadLeft = hardwareMap.dcMotor.get("motor_1"); //Refers to motor...
        motorTreadRight = hardwareMap.dcMotor.get("motor_2");

        //in tik
        motorArm = hardwareMap.dcMotor.get("motorArm");
        //Previously commented out

        /*
        motorController3 = hardwareMap.dcMotorController.get("motorController3");
        motorPullupRight = hardwareMap.dcMotor.get("motor_3");
        motorPullupLeft = hardwareMap.dcMotor.get("motor_4");

        motorController4 = hardwareMap.dcMotorController.get("motorController4");
        motorPivotRight = hardwareMap.dcMotor.get("motor_5");
        motorPivotLeft = hardwareMap.dcMotor.get("motor_6");
        */

        //Setting all motors in forward direction
        motorTreadLeft.setDirection(DcMotor.Direction.FORWARD);
        motorTreadRight.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        //servos in aso
        // leftShovel = hardwareMap.servo.get("leftShovel");
        // rightShovel = hardwareMap.servo.get("rightShovel");
        leftWindshieldWiper = hardwareMap.servo.get("servo_1");
        // rightWindshieldWiper = hardwareMap.servo.get("rightWindshieldWiper");
        shovelVertical1 = hardwareMap.servo.get("servo_3");
        shovelVertical2 = hardwareMap.servo.get("servo_4");
        sideFlap = hardwareMap.servo.get("servo_6");

        //Removed code
    }

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */

    @Override
    public void loop() {

        //Removed code

        //Pushing joystick forward returns negative value
        //If y = -1, the joystick was pushed all of the way forward
        motorPowerTreadLeft = -gamepad1.left_stick_y;
        motorPowerTreadRight = -gamepad1.right_stick_y;

        motorPowerTreadRight = Range.clip(motorPowerTreadRight, -1, 1);
        motorPowerTreadLeft = Range.clip(motorPowerTreadLeft, -1, 1);

        motorPowerTreadRight = (float)scaleInput(motorPowerTreadRight);
        motorPowerTreadLeft = (float)scaleInput(motorPowerTreadLeft);

        motorTreadLeft.setPower(motorPowerTreadLeft);
        motorTreadRight.setPower(motorPowerTreadRight);

        if(gamepad1.left_bumper){
            motorTreadLeft.setDirection(DcMotor.Direction.REVERSE);
            motorTreadRight.setDirection(DcMotor.Direction.FORWARD);
        }else if(gamepad1.left_trigger != 0){
            motorTreadLeft.setDirection(DcMotor.Direction.FORWARD);
            motorTreadRight.setDirection(DcMotor.Direction.REVERSE);
        }

        //Right trigger means that rack and pinion (for collecting mechanism) moves up
        if (gamepad2.right_trigger >= .5) {
            motorPowerArm = 1;
        }else if(gamepad2.right_bumper){
            motorPowerArm = (-1);
        }
        else {
            motorPowerArm = 0;
        }
        motorPowerArm = Range.clip(motorPowerArm, -1, 1);
        motorPowerArm = (float)scaleInput(motorPowerArm);
        motorArm.setPower(motorPowerArm);

        //All commands for "windshield wipers" on shovel
        //Press the two outermost buttons to move "windshield wipers" out
        //Press two innermost buttons for moving in


        if (gamepad2.dpad_left) {
            //Left button on D-pad
            sweeperLeft += 0.1;
            //"Sweeper" or "windshield wiper" on left moves out
            leftWindshieldWiper.setPosition(Range.clip(sweeperLeft, .1, .9));

        }
        if (gamepad2.dpad_right){
            //Right button on D-pad
            sweeperLeft -= 0.1;
            //"Sweeper" or "windshield wiper" on left moves in
            leftWindshieldWiper.setPosition(Range.clip(sweeperLeft, .1, .9));
        }
        //
        /*
        if (gamepad2.b) {
            sweeperRight += 0.1;
        //    rightWindshieldWiper.setPosition(sweeperRight);
        }

        if (gamepad2.x) {
            sweeperRight -= 0.1;
        //    rightWindshieldWiper.setPosition(sweeperRight);
        }
        */
        if (gamepad2.left_bumper)
        {
            // sideFlapB = true;
            // sideFlapPosition += 10;
            sideFlapPosition += 0.1; // Servos go from 0 to 1
            // To change to degrees
            // Scale from 0 to 180
            sideFlap.setPosition(Range.clip(sideFlapPosition, .1, .9));
            //Press right bumper (RB) to move side flap out
        }
        if (gamepad2.left_trigger >= .5){
            // sideFlapPosition -= 10;
            sideFlapPosition -= 0.1;
            sideFlap.setPosition(Range.clip(sideFlapPosition, .1, .9));
        }
        if (gamepad2.dpad_up)
        {
            shovelV1Position += 0.1;
            shovelVertical1.setPosition(Range.clip(shovelV1Position, .1, .9));
        }
        if (gamepad2.dpad_down)
        {
            shovelV1Position -= 0.1;
            shovelVertical1.setPosition(Range.clip(shovelV1Position, .1, .9));
        }
        if (gamepad2.y)
        {
            shovelV2Position += 0.1;
            shovelVertical2.setPosition(Range.clip(shovelV2Position, .1, .9));
        }
        if (gamepad2.a)
        {
            shovelV2Position -= 0.1;
            shovelVertical2.setPosition(Range.clip(shovelV2Position, .1, .9));
        }

        //Removed code
        //Removed code

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        telemetry.addData("Text", "*** Robot Data***");
        // Removed code
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
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

}

