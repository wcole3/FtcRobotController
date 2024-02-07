/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Random Autonomous")
public class RandomAutonomous extends LinearOpMode
{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo wristServo = null;

    private Servo intakeServo = null;
    private Servo hugServo = null;

    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;

    /**
     * Constants
     */

    private final int ARM_TICKS_PER_INPUT = 5;
    // The min motor encoder position observed from trial and error
    private int MIN_ARM_POS = 10;
    // The max motor encoder position observed from trial and error
    private int MAX_ARM_POS = 3000;

    private final double MAX_CLAW_POS = 0.55;
    private final double MIN_CLAW_POS = 0.19;


    private double lastArmPos = 0.0;

    // This also sets the starting position
    private double clawPos = 0.4;
    private double intakePos = 0.26;
    private double wristPos = 0.0;

    private enum VOTE{
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode()
    {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftArmMotor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "right_arm_motor");

        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        hugServo = hardwareMap.get(Servo.class,"hug_servo");
        intakeServo = hardwareMap.get(Servo.class,"intakeServo");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        // Arm motors
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        // must set position before switching mode
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the min arm position to whereever it starts
        MIN_ARM_POS = (int)(leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition())/2;

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        // randomly pick left, right, or center
        VOTE vote = VOTE.CENTER;
        double random = Math.random();
        if(random < 0.33){
            vote = VOTE.LEFT;
        }else if(random < 0.66){
            vote = VOTE.RIGHT;
        }

        // once we get here, we have decided which direction to travel
        // TODO do the movement
        runtime.reset();

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = 0.0;  // forward and reverse
        double lateral =  0.0; // strafe side to side
        double yaw     =  0.0;  // turn robot

        double[] powers = setMotorPowers(axial, lateral, yaw);
        if(vote == VOTE.LEFT){
            // -------Step 1:  Drive forward  ------
            powers = setMotorPowers(0.25, 0.0, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 5.4) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------END STEP 1 ----------------------------

            // ---------- Strafe Left ----------------------------
            powers = setMotorPowers(0.0, -0.25, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------------- END STEP 2---------------------
        }
        else if(vote == VOTE.RIGHT){
            // -------Step 1:  Drive forward  ------
            powers = setMotorPowers(0.25, 0.0, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 5.4) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------END STEP 1 ----------------------------

            // ---------- Strafe Right ----------------------------
            powers = setMotorPowers(0.0, 0.25, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------------- END STEP 2---------------------
        }else{
            // default to center, lowest risk
            // -------Step 1:  Drive forward  ------
            powers = setMotorPowers(0.25, 0.0, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 6.25) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------END STEP 1 ----------------------------
        }
    }

    public double[] setMotorPowers(double axial, double lateral, double yaw){
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }
}
