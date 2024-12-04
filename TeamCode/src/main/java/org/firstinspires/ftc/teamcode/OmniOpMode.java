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

    /** Motor and Servo definitions */
    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftArmMotor = null;
    private DcMotor intakeArmMotor = null;
    private DcMotor intakeInOutMotor = null;
    // Servos
    private Servo liftArmServo = null;
    private Servo liftBucketServo = null;
    private Servo intakeArmServo = null;
    private CRServo intakeBucketServo = null;


    /** Control Vars */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double axial;
    double lateral;
    double yaw;


    /** Control Input */
    boolean slowMode;
    boolean fastMode;
    double intakeArmPosition;
    boolean intakeArmIn;
    boolean intakeArmOut;
    double liftArmMotorPosition;
    boolean liftArmDump;
    boolean liftArmExtend;
    boolean liftArmRetract;
    boolean intakeBucketUp;
    boolean intakeBucketDown;
    boolean intakeBucketspinCW;
    boolean intakeBucketspinCCW;




    /*
     * Constants
     */
    private int liftArmStart;// starting position of the lift arm
    private final int MOTOR_LIMIT = 2700;

    private int intakeArmStart;
    private final int INTAKE_MOTOR_LIMIT = 180;

    private int intakeInOutStart;
    private final int INTAKE_INOUT_MOTOR_LIMIT = 3450;

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
            // Drive motor
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
            // Arm motors
            liftArmMotor = hardwareMap.get(DcMotor.class, "liftArmMotor");
            intakeArmMotor = hardwareMap.get(DcMotor.class, "extendoneck");
            intakeInOutMotor = hardwareMap.get(DcMotor.class, "intakeInOutMotor ");
            //Servos
            liftBucketServo = hardwareMap.get(Servo.class, "liftBucketServo");
            liftArmServo = hardwareMap.get(Servo.class, "liftArmServo");
            intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
            intakeBucketServo = hardwareMap.get(CRServo.class, "intakeBucketServo");

            /*
                Setup motors
             */
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            liftArmStart = liftArmMotor.getCurrentPosition();
            liftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeArmStart = intakeArmMotor.getCurrentPosition();
            intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeInOutStart = intakeInOutMotor.getCurrentPosition();
            intakeInOutMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        handleLiftArm();  // Use input to control lift arm
        handleIntakeArm();
    }

    private void getGamepadInputs(){
        // Get the inputs from the controllers
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;
        slowMode = gamepad1.left_bumper;
        fastMode = gamepad1.right_bumper;
        liftArmMotorPosition = gamepad2.right_stick_y;
        intakeArmPosition = gamepad2.left_stick_y;
        intakeArmOut = gamepad2.left_trigger > 0.5;
        intakeArmIn = gamepad2.left_bumper;
        liftArmDump = gamepad2.square;
        liftArmExtend = gamepad2.triangle;
        liftArmRetract = gamepad2.circle;
        intakeBucketUp = gamepad2.dpad_up;
        intakeBucketDown = gamepad2.dpad_down;
        intakeBucketspinCW = gamepad2.right_trigger > 0.5;
        intakeBucketspinCCW = gamepad2.right_bumper;


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

    private void handleLiftArm(){
        // Logic for controlling the lift arm up/down
        if(liftArmMotorPosition <= 0.0) {
            if(liftArmMotor.getCurrentPosition() > (liftArmStart - 10)){
                liftArmMotor.setPower(liftArmMotorPosition);
            }else{
                liftArmMotor.setPower(0.0);
            }
        }
        else{
            if(liftArmMotor.getCurrentPosition() < (MOTOR_LIMIT + liftArmStart)){
                liftArmMotor.setPower(liftArmMotorPosition);
            }
            else{
                liftArmMotor.setPower(0.0);
                //testMotor.setTargetPosition(MOTOR_LIMIT+beginning);
            }
        }

        // Logic for controlling lift arm servo position
        if(liftArmExtend){
            liftArmServo.setPosition(liftArmServo.getPosition() + 0.01);
            liftBucketServo.setPosition((1.0 - liftArmServo.getPosition()));
        }
        else if(liftArmRetract){
            liftArmServo.setPosition(liftArmServo.getPosition() - 0.01);
            liftBucketServo.setPosition(1.0 - liftArmServo.getPosition());
        }
    }

    private void handleIntakeArm(){
        // Logic to control arm rotation
        if(intakeArmPosition <= 0.0 ){
            if(intakeArmMotor.getCurrentPosition() < intakeArmStart + 100){
                intakeArmMotor.setPower(0.0);
            }else{
                intakeArmMotor.setPower(intakeArmPosition/2.0);
            }
        }
        else{
            if(intakeArmMotor.getCurrentPosition() >= INTAKE_MOTOR_LIMIT){
                intakeArmMotor.setPower(0.0);
            }else{
                intakeArmMotor.setPower(intakeArmPosition/2.0);
            }
        }

        // Logic to control intake arm in/out
        if(intakeArmOut){
            if(intakeInOutMotor.getCurrentPosition() >= INTAKE_INOUT_MOTOR_LIMIT){ // >=)
                intakeInOutMotor.setPower(0.0);
            }
            else{
                intakeInOutMotor.setPower(-0.5);
            }
        }
        else if(intakeArmIn){
            if(intakeInOutMotor.getCurrentPosition() <= intakeInOutStart + 20){ // >=)
                intakeInOutMotor.setPower(0.0);
            }
            else{
                intakeInOutMotor.setPower(1.0);
            }
        }
        else{
            intakeInOutMotor.setPower(0.0);
        }

        // Logic to handle wrist motion
        if(intakeBucketUp){
            intakeArmServo.setPosition(intakeArmServo.getPosition() +0.01);
        }
        else if(intakeBucketDown){
            intakeArmServo.setPosition(intakeArmServo.getPosition() - 0.01);
        }

        if(intakeBucketspinCW){
            intakeBucketServo.setPower(1.0);
        }
        else if(intakeBucketspinCCW){
            intakeBucketServo.setPower(-1.0);
        }
        else{
            intakeBucketServo.setPower(0.0);
        }
    }

    // print out the robot's telemetry
    private void printTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Lift Arm Position: ", liftArmMotor.getCurrentPosition());
        telemetry.addData("Intake Arm Position: ", intakeArmMotor.getCurrentPosition());
        telemetry.addData("Intake In/Out Position: ", intakeInOutMotor.getCurrentPosition());

//        telemetry.add ake servo position", "%4.2f", intakeServo.getPosition());

        telemetry.update();
        // print out the robot's telemetry
        // TODO
    }

}
