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

/*package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoController;


/**
 * TeleOp Code
 * By Rachael H. and Saranya T.
 * Created on 20 November 2015
 */


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
/*
/*
public class K9TankDrive extends OpMode {
 */

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
    // TETRIX VALUES.
    /*final static double ARM_MIN_RANGE  = 0.20;
    final static double ARM_MAX_RANGE  = 0.90;
    final static double CLAW_MIN_RANGE  = 0.20;
    final static double CLAW_MAX_RANGE  = 0.7;
	float left;
	float right;
	float arm1;
	float arm2;


	// position of the arm servo.
	//double armPosition;

	// amount to change the arm servo position.
	//double armDelta = 0.1;

	// position of the claw servo
	//double clawPosition;

	// amount to change the claw servo position by
	//double clawDelta = 0.1;

	float motorPowerArm = 0;
	float motorPowerTreadLeft = 0;
	float motorPowerTreadRight = 0;
	float motorPowerAutonomous = 0;
	float motorPowerPullupLeft = 0;
	float motorPowerPullupRight = 0;
	float motorPowerPivotLeft = 0;
	float motorPowerPivotRight = 0;

	// DcMotorController.DeviceMode devMode;

	DcMotorController MotorController1;
	DcMotor motorTreadRight;
	DcMotor motorTreadLeft;

	DcMotorController MotorController2;
	DcMotor motorArm;
	DcMotor motorAutonomous;

	DcMotorController MotorController3;
	DcMotor motorPullupRight;
	DcMotor motorPullupLeft;

	DcMotorController MotorController4;
	DcMotor motorPivotRight;
	DcMotor motorPivotLeft;

	ServoController ServoController1;
	Servo leftShovel;
	Servo rightShovel;
	Servo leftWindshieldWiper;
	Servo rightWindshieldWiper;
	Servo shovelVertical1;
	Servo shovelVertical2;

	ServoController ServoController2;
	Servo sideFlap;

	//this initializes the boolean
	int reverse = 0;
	boolean up = false;
	// float wristPosition = 0;
	float sweeperLeft = 0;
	float sweeperRight = 0;
	float temp;

	//Servo claw;
	//Servo arm;

	/*
	 * Constructor
	 */
	/*
	public K9TankDrive() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */

	/*
	@Override
	public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */

		/*
		MotorController1 = hardwareMap.dcMotorController.get("MotorController1");
		motorTreadRight = hardwareMap.dcMotor.get("motor_2");
		motorTreadLeft = hardwareMap.dcMotor.get("motor_1");

		MotorController2 = hardwareMap.dcMotorController.get("MotorController2");
		motorArm = hardwareMap.dcMotor.get("motor_7");
		motorAutonomous  = hardwareMap.dcMotor.get("motor_8");

		MotorController3 = hardwareMap.dcMotorController.get("MotorController3");
		motorPullupRight = hardwareMap.dcMotor.get("motor_3");
		motorPullupLeft = hardwareMap.dcMotor.get("motor_4");

		MotorController4 = hardwareMap.dcMotorController.get("MotorController4");
		motorPivotRight = hardwareMap.dcMotor.get("motor_5");
		motorPivotLeft = hardwareMap.dcMotor.get("motor_6");

		motorTreadLeft.setDirection(DcMotor.Direction.FORWARD);
		motorTreadRight.setDirection(DcMotor.Direction.FORWARD);
		motorArm.setDirection(DcMotor.Direction.FORWARD);
		motorAutonomous.setDirection(DcMotor.Direction.FORWARD);
		motorPullupRight.setDirection(DcMotor.Direction.FORWARD);
		motorPullupLeft.setDirection(DcMotor.Direction.FORWARD);
		motorPivotRight.setDirection(DcMotor.Direction.FORWARD);
		motorPivotLeft.setDirection(DcMotor.Direction.FORWARD);

		ServoController1 = hardwareMap.servoController.get("ServoController1");
		leftShovel = hardwareMap.servo.get("leftShovel");
		rightShovel = hardwareMap.servo.get("rightShovel");
		leftWindshieldWiper = hardwareMap.servo.get("leftWindshieldWiper");
		rightWindshieldWiper = hardwareMap.servo.get("rightWindshieldWiper");
		shovelVertical1 = hardwareMap.servo.get("shovelVertical1");
		shovelVertical2 = hardwareMap.servo.get("shovelVertical2");

		ServoController2 = hardwareMap.servoController.get("ServoController2");
		sideFlap = hardwareMap.servo.get("sideFlap");

		/*
		arm = hardwareMap.servo.get("servo_1");
		claw = hardwareMap.servo.get("servo_6");
		*/

		/*
		// assign the starting position of the wrist and claw
		armPosition = 0.2;
		clawPosition = 0.2;
		*/
/*
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
/*
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        /*left = -gamepad1.left_stick_y;
		right = -gamepad1.right_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);
		
		// write the values to the motors
		motorTreadRight.setPower(right);
		motorTreadLeft.setPower(left);

		// True = 1, and false = 0
		temp = gamepad1.left_trigger;
		// Gives a decimal
		temp = Math.round(temp);
		// Rounds to either 0 or 1
		reverse = (int)temp;

		if (reverse == 1) {
			// Special thing for reversing everything
			// Left trigger reverses everything
			// Test whether x must be continually held or only pressed once
			motorTreadLeft.setDirection(DcMotor.Direction.REVERSE);
			motorTreadRight.setDirection(DcMotor.Direction.REVERSE);
		}
		else {
			motorTreadLeft.setDirection(DcMotor.Direction.FORWARD);
			motorTreadRight.setDirection(DcMotor.Direction.FORWARD);
		}

		if (gamepad2.x) {
			motorArm.setPower(1);
			motorPowerArm = 1;
		}

		else {
			motorArm.setPower(0);
		}

		motorPowerTreadLeft = gamepad1.left_stick_y;
		motorPowerTreadRight = gamepad1.right_stick_y;
		motorPowerTreadRight = Range.clip(motorPowerTreadRight, -1, 1);
		motorPowerTreadLeft = Range.clip(motorPowerTreadLeft, -1, 1);

		motorPowerTreadRight = (float)scaleInput(motorPowerTreadRight);
		motorPowerTreadLeft = (float)scaleInput(motorPowerTreadLeft);

		motorTreadLeft.setPower(motorPowerTreadLeft);
		motorTreadRight.setPower(motorPowerTreadRight);

		if (gamepad2.dpad_left) {
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

		/*
		// update the position of the arm.
		if (gamepad1.a) {
			// if the A button is pushed on gamepad1, increment the position of
			// the arm servo.
			// armPosition += armDelta;
		}

		if (gamepad1.y) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the arm servo.
			// armPosition -= armDelta;
		}
		*/

		/*
        // update the position of the claw
        if (gamepad1.left_bumper) {
            // clawPosition += clawDelta;
        }

        if (gamepad1.left_trigger > 0.25) {
            // clawPosition -= clawDelta;
        }

        if (gamepad1.b) {
            clawPosition -= clawDelta;
        }

		// update the position of the claw
		if (gamepad1.x) {
			clawPosition += clawDelta;
		}

		if (gamepad1.b) {
			clawPosition -= clawDelta;
		}

		// clip the position values so that they never exceed their allowed range.
		armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
		clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
		arm.setPosition(armPosition);
		claw.setPosition(clawPosition);
		*/

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

		/*telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
        telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
		telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	/*@Override
	public void stop() {

	}
	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	/*double scaleInput(double dVal)  {
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

}*/
