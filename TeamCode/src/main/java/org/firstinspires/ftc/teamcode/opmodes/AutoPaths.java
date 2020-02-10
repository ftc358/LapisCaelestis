package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.TRACK_WIDTH;


@Config
public class AutoPaths {

    public static double PARK_X = 0;
    public static double PARK_Y_RED = -44;

    public enum Alliance {
        RED,
        BLUE
    }

    public enum Side {
        DEPOT,
        FOUNDATION
    }

    public static Alliance ALLIANCE;

    public static Side SIDE;

    public static AutoOpmode.StonePosition STONE_POSITION = AutoOpmode.StonePosition.LEFT;

    public static DriveConstraints constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);

    public static TrajectoryBuilder trajectoryBuilder(Pose2d pose) {
        return new TrajectoryBuilder(pose, constraints);
    }

    //    public static Trajectory makeInitialToAlign(Pose2d pose) {
//        switch (ALLIANCE) {
//            case RED: {
//                switch (STONE_POSITION) {
//                    case LEFT:
//                        return trajectoryBuilder(pose)
//                                .splineTo(new Pose2d(-26.0, -38.0, toRadians(90.0)))
//                                .build();
//                    case MIDDLE:
//                        return trajectoryBuilder(pose)
//                                .splineTo(new Pose2d(-20.0, -36.0, toRadians(90.0)))
//                                .build();
//                    case RIGHT:
//                        return trajectoryBuilder(pose)
//                                .splineTo(new Pose2d(-32.0, -25.0, toRadians(20.0)), new SplineInterpolator(toRadians(90.0), toRadians(20.0)))
//                                .build();
//                }
//            }
//            case BLUE: {
//                // TODO
//            }
//        }
//        return null;
//    }
//
//    public static Trajectory makeAlignToAcquisition1(Pose2d pose) {
//        switch (ALLIANCE) {
//            case RED: {
//                switch (STONE_POSITION) {
//                    case LEFT:
//                        return trajectoryBuilder(pose)
//                                .strafeTo(new Vector2d(-26.0, -30.0))
//                                .lineTo(new Vector2d(-30.0,-21.0))
//                                .build();
//                    case MIDDLE:
//                        return trajectoryBuilder(pose)
//                                .strafeTo(new Vector2d(-20.0, -21.0))
//                                .lineTo(new Vector2d(-30.0,-21.0))
//                                .build();
//                    case RIGHT:
//                        return trajectoryBuilder(pose)
//                                .splineTo(new Pose2d(-48.0, -36.0, toRadians(0.0)), new SplineInterpolator(toRadians(90.0), toRadians(0.0)))
//                                .strafeTo(new Vector2d(-48.0,-8.0))
//                                .splineTo(new Pose2d(-42.0,-21.0,toRadians(0.0)), new ConstantInterpolator(toRadians(0.0)))
//                                .build();
//                }
//            }
//            case BLUE: {
//                // TODO
//            }
//        }
//        return null;
//    }
//
//    public static Trajectory makeDeliverFirstStone(Pose2d pose) {
//        switch (ALLIANCE) {
//            case RED: {
//                switch (STONE_POSITION) {
//                    case LEFT:
//                        return trajectoryBuilder(pose)
//                                .setReversed(true)
//                                .splineTo(new Pose2d(-10.0, -36.0, toRadians(180.0)))
//                                .splineTo(new Pose2d(61.0, -32.0, toRadians(270.0)))
//                                .build();
//                    case MIDDLE:
//                        return trajectoryBuilder(pose)
//                                .setReversed(true)
//                                .splineTo(new Pose2d(-10.0, -36.0, toRadians(180.0)))
//                                .splineTo(new Pose2d(61.0, -32.0, toRadians(270.0)))
//                                .build();
//                    case RIGHT:
//                        return trajectoryBuilder(pose)
//                                .splineTo(new Pose2d(-10.0, -36.0, toRadians(180.0)))
//                                .splineTo(new Pose2d(61.0, -32.0, toRadians(270.0)))
//                                .build();
//                }
//            }
//            case BLUE: {
//                // TODO
//            }
//        }
//        return null;
//    }
//
//    public static Trajectory makeFoundationToStone(Pose2d pose) {
//        switch (ALLIANCE) {
//            case RED: {
//                switch (STONE_POSITION) {
//                    case LEFT:
//                        return trajectoryBuilder(pose)
//                                .setReversed(false)
//                                .splineTo(new Pose2d(10.0, -36.0, toRadians(180.0)))
//                                .splineTo(new Pose2d(-60.0, -24.0, toRadians(160.0)))
//                                .build();
//                    case MIDDLE:
//                        return trajectoryBuilder(pose)
//                                .setReversed(false)
//                                .splineTo(new Pose2d(10.0, -36.0, toRadians(180.0)))
//                                .splineTo(new Pose2d(-52.0, -24.0, toRadians(160.0)))
//                                .build();
//                    case RIGHT:
//                        return trajectoryBuilder(pose)
//                                .setReversed(false)
//                                .splineTo(new Pose2d(-30.0, -36.0, toRadians(180.0)))
//                                .splineTo(new Pose2d(-46.0, -24.0, toRadians(160.0)))
//                                .build();
//                }
//            }
//            case BLUE: {
//                // TODO
//            }
//        }
//        return null;
//    }
//
//    public static Trajectory makeDeliverSecondStone(Pose2d pose2d) {
//        switch (ALLIANCE) {
//            case RED: {
//                return trajectoryBuilder(pose2d)
//                        .setReversed(true)
//                        .splineTo(new Pose2d(0.0, -36.0, toRadians(180.0)))
//                        .splineTo(new Pose2d(58.0, -32.0, toRadians(270.0)))
//                        .build();
//            }
//            case BLUE: {
//                // TODO
//            }
//        }
//        return null;
//    }
//
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

//    public static Trajectory makeStartToFoundation(Pose2d pose) {
//        return trajectoryBuilder(pose)
//                .splineTo(new Pose2d(-10.0, -36.0, toRadians(0.0)))
//                .splineTo(new Pose2d(50.0, -55.0, toRadians(270.0)))
//                .setReversed(true)
//                .splineTo(new Pose2d(61.0, -32.0, toRadians(270.0)))
//                .build();
//    }

    public static Trajectory makeFoundationToPark(Pose2d pose) {
        switch (ALLIANCE) {
            case RED: {
                return trajectoryBuilder(pose)
                        .splineTo(new Pose2d(0.0, -44.0, toRadians(180.0)))
                        .build();
            }
            case BLUE: {
                return trajectoryBuilder(pose)
                        .splineTo(new Pose2d(0.0, 44.0, toRadians(180.0)))
                        .build();
            }
        }
        return null;


    }

    public static Trajectory makeStartToPark(Pose2d pose) {
        double finalHeading = SIDE == Side.DEPOT ? 0.0 : 180.0;
        switch (ALLIANCE) {
            case RED: {
                return trajectoryBuilder(pose)
                        .splineTo(new Pose2d(PARK_X, PARK_Y_RED, toRadians(finalHeading)))
                        .build();
            }
            case BLUE: {
                return trajectoryBuilder(pose)
                        .splineTo(new Pose2d(PARK_X, -PARK_Y_RED, toRadians(finalHeading)))
                        .build();
            }
        }
        return null;
    }
}
