package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.TRACK_WIDTH;

public class AutoPaths {

    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance ALLIANCE;

    public static AutoOpmode.StonePosition STONE_POSITION = AutoOpmode.StonePosition.LEFT;

    public static DriveConstraints constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);

    public static TrajectoryBuilder trajectoryBuilder(Pose2d pose) {
        return new TrajectoryBuilder(pose, constraints);
    }

    public static double LEFT_STONE_X = -32.0;
    public static double MIDDLE_STONE_X = -30.0;

//    public static Trajectory makeInitialToDetect(Pose2d pose) {
//        switch (ALLIANCE) {
//            case RED: {
//                return trajectoryBuilder(pose)
//                        .splineTo(new Pose2d(-36.0, -48.0, toRadians(90)))
//                        .build();
//            }
//            case BLUE: {
//                // TODO
//                return trajectoryBuilder(pose)
//                        .splineTo(new Pose2d(-36.0, 48.0, toRadians(270)))
//                        .build();
//            }
//        }
//        return null;
//    }

    public static Trajectory makeInitialToStone(Pose2d pose) {
        switch (ALLIANCE) {
            case RED: {
                switch (STONE_POSITION) {
                    case LEFT:
                        return trajectoryBuilder(pose)
                                .splineTo(new Pose2d(-32.0, -24.0, toRadians(160.0)), new SplineInterpolator(toRadians(90.0), toRadians(160.0)))
                                .build();
                    case MIDDLE:
                        return trajectoryBuilder(pose)
                                .splineTo(new Pose2d(-22.0, -30.0, toRadians(90.0)), new ConstantInterpolator(toRadians(90.0)))
                                .splineTo(new Pose2d(-30.0, -24.0, toRadians(160.0)), new SplineInterpolator(toRadians(90.0), toRadians(160.0)))
                                .build();
                    case RIGHT:
                        return trajectoryBuilder(pose)
                                .splineTo(new Pose2d(-32.0, -25.0, toRadians(20.0)), new SplineInterpolator(toRadians(90.0), toRadians(20.0)))
                                .build();
                }
            }
            case BLUE: {
                // TODO
            }
        }
        return null;
    }

    public static Trajectory makeDeliverFirstStone(Pose2d pose) {
        switch (ALLIANCE) {
            case RED: {
                return trajectoryBuilder(pose)
                        .setReversed(true)
                        .splineTo(new Pose2d(-10.0, -36.0, toRadians(180.0)))
                        .build();
            }
            case BLUE: {
                // TODO
            }
        }
        return null;
    }

    public static Trajectory makeFoundationToStone(Pose2d pose) {
        switch (ALLIANCE) {
            case RED: {
                switch (STONE_POSITION) {
                    case LEFT:
                        return trajectoryBuilder(pose)
                                .setReversed(false)
                                .splineTo(new Pose2d(10.0, -36.0, toRadians(180.0)))
                                .splineTo(new Pose2d(-60.0, -24.0, toRadians(160.0)))
                                .build();
                    case MIDDLE:
                        return trajectoryBuilder(pose)
                                .setReversed(false)
                                .splineTo(new Pose2d(10.0, -36.0, toRadians(180.0)))
                                .splineTo(new Pose2d(-52.0, -24.0, toRadians(160.0)))
                                .build();
                    case RIGHT:
                        return trajectoryBuilder(pose)
                                .setReversed(false)
                                .splineTo(new Pose2d(-30.0, -36.0, toRadians(180.0)))
                                .splineTo(new Pose2d(-46.0, -24.0, toRadians(160.0)))
                                .build();
                }
            }
            case BLUE: {
                // TODO
            }
        }
        return null;
    }

    public static Trajectory makeDeliverSecondStone(Pose2d pose2d) {
        switch (ALLIANCE) {
            case RED: {
                return trajectoryBuilder(pose2d)
                        .setReversed(true)
                        .splineTo(new Pose2d(0.0, -36.0, toRadians(180.0)))
                        .splineTo(new Pose2d(58.0, -32.0, toRadians(270.0)))
                        .build();
            }
            case BLUE: {
                // TODO
            }
        }
        return null;
    }

    public static Trajectory makeMoveFoundation(Pose2d pose2d) {
        switch (ALLIANCE) {
            case RED: {
                return trajectoryBuilder(pose2d)
                        .setReversed(false)
                        .splineTo(new Pose2d(36.0, -50.0, toRadians(180.0)))
                        .build();
            }
            case BLUE: {
                // TODO
            }
        }
        return null;
    }
}
