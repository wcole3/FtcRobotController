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
import com.qualcomm.robotcore.hardware.DcMotor;
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

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

//    private Servo intakeServo = null;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double axial;
    double lateral;
    double yaw;
    boolean modifier;
    boolean modifier2;
    boolean Abutten;
    boolean Bbutten;
    boolean launch;

    double hugServoOpen;
    double hugServoClose;
    boolean wristServoOpen;
    boolean wristServoClose;
    double testArmMotorPosition;
    /*
     * Constants
     */

    private double intakePos = 0.12;

    //LOL
    @Override
    public void runOpMode() {

        // initialize Robot
        if(!initializeRobot()){

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
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
//            intakeServo = hardwareMap.get(Servo.class, "intakeServo");

            /*
                Setup motors
             */
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
//        handleAttachmentMotion(); // Use inputs to control attachments
    }

    private void getGamepadInputs(){
        // Get the inputs from the controllers
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;
        modifier = gamepad1.left_bumper;
        modifier2 = gamepad1.right_bumper;
        Abutten = gamepad1.a;
        Bbutten = gamepad1.b;
        launch = gamepad2.a && gamepad2.b;
        hugServoOpen = gamepad2.left_trigger;
        hugServoClose = gamepad2.right_trigger;
        wristServoOpen = gamepad2.left_bumper;
        wristServoClose = gamepad2.right_bumper;
        testArmMotorPosition = gamepad2.right_stick_y;
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
        if (modifier) {
            leftFrontPower *= 0.25;
            leftBackPower *= 0.25;
            rightBackPower *= 0.25;
            rightFrontPower *= 0.25;
        }
        // If Turbo is not pressed clip speed to 50% max
        if (!modifier2) {
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

//    private void handleAttachmentMotion() {
//        // intake servo
//        if (Abutten) {
//            intakePos = 0.12;
//            intakeServo.setPosition(intakePos);
//        } else if (Bbutten) {
//            intakePos = 0.09;
//            intakeServo.setPosition(intakePos);
//        } else {
//            intakeServo.setPosition(intakePos);
//        }
//    }

    // print out the robot's telemetry
    private void printTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

//        telemetry.add ake servo position", "%4.2f", intakeServo.getPosition());

        telemetry.update();
        // print out the robot's telemetry
        // TODO
    }

}
