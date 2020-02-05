package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.subsystems.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.concurrent.ExecutorService;

@TeleOp
public class MinimalFollowTrajectoryDemo extends LinearOpMode {

    SampleMecanumDriveREVOptimized drive;
    private ExecutorService driveUpdateExecutor;
    private Runnable driveUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            drive.update(null);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        driveUpdateExecutor = ThreadPool.newSingleThreadExecutor("drive update");
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        driveUpdateExecutor.submit(driveUpdateRunnable);

        Trajectory testTrajectory = drive.trajectoryBuilder()
                .forward(20)
                .build();

       drive.followTrajectory(testTrajectory);

       Thread.sleep(2000);

       driveUpdateExecutor.shutdown();
    }
}
