/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
// lol

@TeleOp(name = "Omni OpMode", group = "Linear OpMode")
//@Disabled
public class OmniOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Motor and Servo definitions
     */
    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor bigWheel = null;
    private DcMotor littleWheel = null;
    // Servos
    private CRServo hopperServo;//

    /**
     * Control Vars
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double axial;
    double lateral;
    double yaw;
    long bigWheelStart;


    /**
     * Control Input
     */
    boolean slowMode = false;
    boolean fastMode = false;
    boolean hopperRunning = false;
    double bigWheelSpeed = 0.65;
    private long timeElapsed = 0;


    /*
     * Constants
     */

    //LOL
    @Override
    public void runOpMode() {

        // initialize Robot
        if (!initializeRobot()) {
            // if we get here, something went very, VERY, wrong!!!!!
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            controlRobot();

            printTelemetry();

        }
    }

    private boolean initializeRobot() {
        // Setup all variables from the hardmap
        boolean status = false;
        try {
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            // Drive motor
            leftFrontDrive = hardwareMap.get(DcMotor.class, "Frontleft");
            leftBackDrive = hardwareMap.get(DcMotor.class, "Backleft");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "Frontright");
            rightBackDrive = hardwareMap.get(DcMotor.class, "Backright");
            bigWheel = hardwareMap.get(DcMotor.class, "bigWheel");
            littleWheel = hardwareMap.get(DcMotor.class, "littleWheel");
            hopperServo = hardwareMap.get(CRServo.class, "feederServo");//

            /*
                Setup motors
             */
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            bigWheel.setDirection(DcMotor.Direction.REVERSE);
            littleWheel.setDirection(DcMotor.Direction.REVERSE);

            status = true;
        } catch (Exception e) {
            status = false;
        }
        return status;
    }

    private void controlRobot() {
        // Method that control all robot behaviors
        getGamepadInputs(); // Get the inputs from the gamepads
        handleRobotMotion(); // Use inputs to control motion
    }

    private void getGamepadInputs() {
        // Get the inputs from the controllers
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        axial = -gamepad1.right_stick_y;  // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;
        // player1
        if(gamepad1.left_bumper){
            slowMode = true;
            fastMode = false;
        }
        if(gamepad1.left_trigger > 0.5){
            slowMode = false;
            fastMode = false;
        }
        if(gamepad1.right_bumper){
            fastMode = true;
            slowMode = false;
        }
        if(gamepad1.right_trigger > 0.5){
            fastMode = false;
            slowMode = false;
        }

        // player 2
        if(gamepad2.a){
            if(bigWheelStart != -1) {
                if(timeElapsed >  2500) {
                    littleWheel.setPower(1.0);
                } else{
                    littleWheel.setPower(0.0);
                }
            }else{
                littleWheel.setPower(0.0);
            }
        }
        else{
            littleWheel.setPower(0.0);
        }
        if(gamepad2.dpadUpWasReleased()){
            bigWheelSpeed = bigWheelSpeed + 0.05;
            if(bigWheelSpeed > 0.7){
                bigWheelSpeed = 0.7;
            }
        }
        if(gamepad2.dpadDownWasReleased()){
            bigWheelSpeed=bigWheelSpeed - 0.05;
            if(bigWheelSpeed < 0.5){
                bigWheelSpeed = 0.5;
            }
        }

        if(gamepad2.right_trigger > 0.0){
            bigWheel.setPower(bigWheelSpeed);

            if(bigWheelStart == -1){
                bigWheelStart = System.currentTimeMillis();
            }else{
                timeElapsed = System.currentTimeMillis() - bigWheelStart;
            }
        } else{
            bigWheel.setPower(0.0);
            timeElapsed = 0;
            bigWheelStart = -1;
        }

        if(gamepad2.yWasReleased()){
            hopperRunning = !hopperRunning;
            hopperServo.setPower(hopperRunning ? 1.0 : 0.0);//
        }
    }

    private void handleRobotMotion() {

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Slow Mode
        if (slowMode) {
            leftFrontPower *= 0.25;
            leftBackPower *= 0.25;
            rightBackPower *= 0.25;
            rightFrontPower *= 0.25;
        }
        // If Turbo is not pressed clip speed to 50% max
        if (!fastMode) {
            leftFrontPower = Range.clip(leftFrontPower, -0.5, 0.5);
            rightFrontPower = Range.clip(rightFrontPower, -0.5, 0.5);
            leftBackPower = Range.clip(leftBackPower, -0.5, 0.5);
            rightBackPower = Range.clip(rightBackPower, -0.5, 0.5);
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }



    private void setMotorPosition(DcMotor motor, double motorinput, int motor_limit, int lower_limit) {
        // basically the same as
        if(motor.isBusy() || motorinput == 0.0) {
            // TODO need to reset mode; break if into two
            return;
        }
        if(motorinput > 0.0){
            motor.setTargetPosition((int)(motorinput* motor_limit));

        } else if (motorinput < 0.0) {
            motor.setTargetPosition(lower_limit);
        }
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.25);
    }

    // print out the robot's telemetry
    private void printTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Big Wheel Speed: ", "%1.2f", bigWheelSpeed);
        telemetry.addData("timeforRtrigger: ", "%1.2f", timeElapsed/1000.);
        telemetry.update();
    }

}
