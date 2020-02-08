package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opmodes.AutoOpmode;
import org.firstinspires.ftc.teamcode.opmodes.AutoPaths;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import static java.lang.Math.toRadians;

@Autonomous
public class RedDepot extends AutoOpmode {

    @Override
    protected void setup() {
        initialPose = new Pose2d(-36.0, -63.0, toRadians(90.0));
        robot.drive.setPoseEstimate(initialPose);
        AutoPaths.ALLIANCE = AutoPaths.Alliance.RED;
    }

    @Override
    protected void run() {

        stonePosition = skystonePipeline.getPosition();
        AutoPaths.STONE_POSITION = stonePosition;

        RobotLog.a("**************************");
        RobotLog.a(stonePosition.toString());

        Trajectory startToDetect = AutoPaths.makeInitialToStone(robot.drive.getPoseEstimate());
        robot.drive.followTrajectory(startToDetect);
        robot.intake.setPower(1.0, Intake.IntakeMode.OUTTAKE);
        sleep(2000);
        robot.intake.setPower(1.0, Intake.IntakeMode.INTAKE);
        robot.drive.waitForIdle();
    }
}
