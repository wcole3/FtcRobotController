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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name = "Omni Drive Mode", group = "Linear OpMode")
//@Disabled
public class OmniDriveMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Motor and Servo definitions
     */
    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;


    /**
     * Control Vars
     */
    double forward;
    double reverse;
    double turning;

    String debugStr = "";

    //LOL
    @Override
    public void runOpMode() {

        // initialize Robot
        if (!initializeRobot()) {
            // if we get here, something went wrong
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
            leftBackDrive = hardwareMap.get(DcMotor.class, "motor 1");
            rightBackDrive = hardwareMap.get(DcMotor.class, "motor 0");
            rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

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
        forward = gamepad1.right_trigger;
        turning = gamepad1.left_stick_x;
        reverse = gamepad1.left_trigger;
    }

    private void handleRobotMotion(){
        // if forward is being held and reverse gets pressed it keeps going forward.  If reverse is being held and forward is pressed it keeps going in reverse
        double totalRightPower = forward - reverse;
        double totalLeftPower = forward - reverse;
        if(turning > 0.0){
            debugStr = "turning right";
            totalLeftPower = turning;
            totalRightPower = -1.*turning;
        }else if(turning < 0.0){
            debugStr = "turning left";
            // when we are here turning is negative
            totalLeftPower = turning;
            totalRightPower = -1.*turning;
        }else{
            debugStr = "";
        }

//        totalRightPower += turning > 0.0 ? -turning : turning;
//        totalLeftPower += turning > 0.0 ? -turning : turning;

        leftBackDrive.setPower(totalLeftPower);
        rightBackDrive.setPower(totalRightPower);
//        if(gamepad1.right_trigger > 0.0){// if forward is pressed
//            leftBackDrive.setPower(forward);
//            rightBackDrive.setPower(forward);
//        }
//        else if(gamepad1.left_trigger > 0.0){// if reverse is pressed
//            leftBackDrive.setPower(reverse);
//            rightBackDrive.setPower(reverse);
//        }


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
        telemetry.addData("Front left/Right", "%4.2f,  %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
        telemetry.addData("Controller Input: ", "%3.2f", turning);
        telemetry.addData("debugstr: ","%s", debugStr);
//        telemetry.addData("Lift Arm Position: ", liftArmMotor.getCurrentPosition());
//        telemetry.addData("Intake Arm 1 Position: ", intakeArmMotor1.getCurrentPosition());
//        telemetry.addData("Intake Arm 2 Position: ", intakeArmMotor2.getCurrentPosition());
//        telemetry.addData("Intake In/Out Position: ", intakeInOutMotor.getCurrentPosition());

        telemetry.update();
    }

}
