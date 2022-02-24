

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Auto extends LinearOpMode {
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
    private int ArmHeightOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        ShubaMotor = hardwareMap.get(DcMotor.class, "ShubaMotor");
        ClawLeft = hardwareMap.get(Servo.class, "ClawLeft");
        ClawRight = hardwareMap.get(Servo.class, "ClawRight");
        KanataServo = hardwareMap.get(Servo.class,"KanataServo");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        KanatArm = hardwareMap.get(DcMotor.class, "KanatArm");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-31.5, 62.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

//      Close claw on init
        ClawLeft.setDirection(Servo.Direction.FORWARD);
        ClawRight.setDirection(Servo.Direction.FORWARD);
        ClawLeft.setPosition(0.75);
        ClawRight.setPosition(0.25);
        ArmMotor.setPower(0);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-8, 50), Math.toRadians(270))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(-60, 36), Math.toRadians(270))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(-67, 50), Math.toRadians(270), SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-69, 24), Math.toRadians(270))
                .build();

//      Move motor
//        ArmMotor.getCurrentPosition();
//        ArmMotor.setTargetPosition(-1700 + ArmHeightOffset);
//        while (ArmMotor.getCurrentPosition() > ArmMotor.getTargetPosition()) {
//            ArmMotor.setPower(-0.5);
//            KanataServo.setPosition(0.88);
//        }

        ArmMotor.setPower(0);

        drive.followTrajectory(traj);

        ArmMotor.setPower(0);
        sleep(500);
        ArmMotor.setPower(0);

//      Move motor
        ArmMotor.getCurrentPosition();
        ArmMotor.setTargetPosition(-2700 + ArmHeightOffset);
        while (ArmMotor.getCurrentPosition() > ArmMotor.getTargetPosition()) {
            ArmMotor.setPower(-0.5);
            KanataServo.setDirection(Servo.Direction.FORWARD);
            KanataServo.setPosition(0.88);
        }

        ArmMotor.setPower(0);

        sleep(500);

//      Open claw
        ClawLeft.setDirection(Servo.Direction.FORWARD);
        ClawRight.setDirection(Servo.Direction.FORWARD);
        ClawLeft.setPosition(0.60);
        ClawRight.setPosition(0.40);

        ArmMotor.setPower(0);

        sleep(500);

        KanataServo.setPosition(1);

        ArmMotor.setTargetPosition(-2400);
        while (ArmMotor.getCurrentPosition() < ArmMotor.getTargetPosition()) {
            ArmMotor.setPower(0.3);
            KanataServo.setDirection(Servo.Direction.REVERSE);
            KanataServo.setPosition(0.28);
        }

        sleep(500);

//      Close Claw
        ClawLeft.setDirection(Servo.Direction.FORWARD);
        ClawRight.setDirection(Servo.Direction.FORWARD);
        ClawLeft.setPosition(0.75);
        ClawRight.setPosition(0.25);
        ArmMotor.setPower(0);

        sleep(500);

        ArmMotor.setTargetPosition(-1500);
            while (ArmMotor.getCurrentPosition() < ArmMotor.getTargetPosition()) {
                ArmMotor.setPower(0.3);
                KanataServo.setPosition(0.28);
            }

        ArmMotor.setPower(0);

        KanataServo.setPosition(0.8);

        sleep(500);

        drive.followTrajectory(traj1);
        ArmMotor.setPower(0);
        drive.followTrajectory(traj2);
        ArmMotor.setPower(0);
        sleep(500);
        ShubaMotor.setPower(0.5);
        sleep(2000);
        ShubaMotor.setPower(0);
        ArmMotor.setPower(0);
        sleep(500);
        drive.followTrajectory(traj3);
        ArmMotor.setPower(0);
    }
}