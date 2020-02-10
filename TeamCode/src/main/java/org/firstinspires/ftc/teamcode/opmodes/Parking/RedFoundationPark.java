package org.firstinspires.ftc.teamcode.opmodes.Parking;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoOpmode;
import org.firstinspires.ftc.teamcode.opmodes.AutoPaths;

import static java.lang.Math.toRadians;

@Autonomous
public class RedFoundationPark extends AutoOpmode {
    @Override
    protected void setup() {
//        initialPose = new Pose2d(36.0, -63.0, toRadians(90.0));
//        robot.drive.setPoseEstimate(initialPose);
//        AutoPaths.ALLIANCE = AutoPaths.Alliance.RED;
//        AutoPaths.SIDE = AutoPaths.Side.FOUNDATION;
    }

    @Override
    protected void run() {

//        stonePosition = skystonePipeline.getPosition();
//        AutoPaths.STONE_POSITION = stonePosition;
//
//        RobotLog.a("**************************");
//        RobotLog.a(stonePosition.toString());
//        telemetry.addLine("stone position " + stonePosition.toString());
//        telemetry.update();

//        Trajectory startToPark = AutoPaths.makeStartToPark(robot.drive.getPoseEstimate());
////        robot.drive.followTrajectorySync(startToPark);

        robot.drive.followTrajectorySync(
                robot.drive.trajectoryBuilder()
                        .strafeLeft(4)
                        .build()
        );
    }
}
