package org.firstinspires.ftc.teamcode.subsystems.drive.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.mecanum.MecanumDriveREVOptimized;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    private Robot robot;
    public static double DISTANCE = 30;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        robot.start();

        robot.drive.setPoseEstimate(new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            robot.drive.followTrajectorySync(
                    robot.drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            robot.drive.turnSync(Math.toRadians(90));
        }
    }
}