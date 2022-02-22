/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Encoder;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP", group="Linear Opmode")
public class TeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor ArmMotor = null;
    private DcMotor KanatArm = null;
    private DcMotor ShubaMotor = null;
    private Servo ClawLeft = null;
    private Servo ClawRight = null;
    private Servo KanataServo = null;
    private Encoder leftEncoder = null;
    private Encoder rightEncoder = null;
    private Encoder frontEncoder = null;
    private Encoder armEncoder = null;
    private int ArmHeight = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        rightRear = hardwareMap.get(DcMotor.class, "RightRear");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        KanatArm = hardwareMap.get(DcMotor.class, "KanatArm");
        ShubaMotor = hardwareMap.get(DcMotor.class, "ShubaMotor");
        ClawLeft = hardwareMap.get(Servo.class, "ClawLeft");
        ClawRight = hardwareMap.get(Servo.class, "ClawRight");
        KanataServo = hardwareMap.get(Servo.class,"KanataServo");
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RightRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RightFront"));

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LeftRear"));
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // left stick controls direction
            // right stick X controls rotation

            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            // holonomic formulas

            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            // write the values to the motors
            rightFront.setPower(FrontRight);
            leftFront.setPower(FrontLeft);
            leftRear.setPower(BackLeft);
            rightRear.setPower(BackRight);
            /*
            //Old Arm Code

            if(gamepad1.right_bumper)
            {
                ArmMotor.setPower(-1);
            }
            else if(gamepad1.left_bumper)
            {
                ArmMotor.setPower(1);
            }
            else
            {
                ArmMotor.setPower(0);
                ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }*/

            //New arm code

            if(gamepad1.dpad_right){
                ArmHeight = 0;
            }
            if(gamepad1.dpad_down){
                ArmHeight = 1;
            }
            if(gamepad1.dpad_left){
                ArmHeight = 2;
            }
            if(gamepad1.dpad_up){
                ArmHeight = 3;
            }

            if(ArmHeight == 0){
                ArmMotor.setTargetPosition(-40);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.5);
                KanataServo.setPosition(0.85);
            }else if(ArmHeight == 1){
                ArmMotor.setTargetPosition(-3500);
                KanataServo.setPosition(0.96);
            }else if(ArmHeight == 2) {
                ArmMotor.setTargetPosition(-3220);
                KanataServo.setPosition(0.91);
            }else{
                ArmMotor.setTargetPosition(-2900);
                KanataServo.setPosition(0.88);
            }

            if(gamepad1.x)
            {
                ShubaMotor.setPower(0.7);
            }
            if(gamepad1.a)
            {
                ClawLeft.setDirection(Servo.Direction.FORWARD);
                ClawRight.setDirection(Servo.Direction.FORWARD);
                ClawLeft.setPosition(0.60);
                ClawRight.setPosition(0.40);
            }
            if(gamepad1.b)
            {
                ClawLeft.setDirection(Servo.Direction.FORWARD);
                ClawRight.setDirection(Servo.Direction.FORWARD);
                ClawLeft.setPosition(0.75);
                ClawRight.setPosition(0.25);
            }
            else
            {
                ShubaMotor.setPower(0);
            }

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("RawLeft", "Raw Left: " + leftEncoder.getCurrentPosition());
            telemetry.addData("RawRight", "Raw Right: " + rightEncoder.getCurrentPosition());
            telemetry.addData("RawFront", "Raw Front: " + frontEncoder.getCurrentPosition());
            telemetry.addData("ArmHeight", "Arm Height: " + ArmMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
