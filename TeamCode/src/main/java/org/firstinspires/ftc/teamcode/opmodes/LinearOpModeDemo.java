package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@TeleOp
public class LinearOpModeDemo extends LinearOpMode {

    private Robot robot;
    private StickyGamepad stickyGamepad;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        robot.start();

        stickyGamepad = new StickyGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");

        waitForStart();

        Trajectory testTrajectory = robot.drive.trajectoryBuilder()
                .forward(20)
                .build();

        while (!isStopRequested()) {
            telemetry.addData("drive mode", robot.drive.mode);
            telemetry.addData("currentPose", robot.drive.getPoseEstimate());
            telemetry.addData("trajectory", testTrajectory);
            robot.drive.followTrajectorySync(testTrajectory);
            telemetry.addData("drive signal", robot.drive.debug);
            telemetry.addData("isFollowing", robot.drive.follower.isFollowing());
            telemetry.addData("motor1", robot.drive.leftFront.getPower());
            telemetry.addData("motor2", robot.drive.rightFront.getPower());
            telemetry.addData("motor3", robot.drive.leftRear.getPower());
            telemetry.addData("motor4", robot.drive.rightRear.getPower());
            telemetry.update();
            robot.foundationGrabber.setServoPosition(gamepad1.left_trigger);
            robot.capstoneFeeder.setServoPosition(0.95 - gamepad1.right_trigger);
        }
    }
}
