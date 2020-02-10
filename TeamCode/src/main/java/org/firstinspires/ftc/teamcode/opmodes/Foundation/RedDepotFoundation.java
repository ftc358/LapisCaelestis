package org.firstinspires.ftc.teamcode.opmodes.Foundation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.opmodes.AutoOpmode;
import org.firstinspires.ftc.teamcode.opmodes.AutoPaths;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;

import static java.lang.Math.toRadians;

@Config
public class RedDepotFoundation extends AutoOpmode {

    public static double firstSegment = 20.0;
    public static double secondSegment = 92.0;

    @Override
    protected void setup() {
        initialPose = new Pose2d(-36.0, -63.0, toRadians(90.0));
        robot.drive.setPoseEstimate(initialPose);
        AutoPaths.ALLIANCE = AutoPaths.Alliance.RED;
        AutoPaths.SIDE = AutoPaths.Side.FOUNDATION;
    }

    @Override
    protected void run() {
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(firstSegment)
                        .build()
        );
        robot.drive.turnSync(toRadians(-90));
        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .forward(secondSegment)
                        .build()
        );
        robot.drive.turnSync(toRadians(-90));
        while (!robot.foundationGrabber.foundationSwitchEngaged) {
            robot.drive.setDrivePower(new Pose2d(-0.1, 0, 0));
        }
        robot.foundationGrabber.setGrabberPosition(FoundationGrabber.GrabberPosition.LOWERED);
        Trajectory moveFoundation = AutoPaths.makeMoveFoundation(robot.drive.getPoseEstimate());
        robot.drive.followTrajectorySync(moveFoundation);
        robot.foundationGrabber.setGrabberPosition(FoundationGrabber.GrabberPosition.STOWED);
        Trajectory foundationToPark = AutoPaths.makeFoundationToPark(robot.drive.getPoseEstimate());
        robot.drive.followTrajectorySync(foundationToPark);
    }
}
