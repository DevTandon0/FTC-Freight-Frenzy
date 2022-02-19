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

    @Override
    public void runOpMode() throws InterruptedException {

        ShubaMotor = hardwareMap.get(DcMotor.class, "ShubaMotor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-31.5, 62.5, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, 42), Math.toRadians(270))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(-60, 36), Math.toRadians(270))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(-59, 52.5), Math.toRadians(270))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-66, 24), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj);
        sleep(500);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        sleep(500);
        ShubaMotor.setPower(0.6);
        sleep(2000);
        ShubaMotor.setPower(0);
        sleep(500);
        drive.followTrajectory(traj3);
    }
}
