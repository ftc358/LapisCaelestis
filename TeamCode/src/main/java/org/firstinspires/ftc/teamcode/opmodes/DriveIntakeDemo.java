package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@Config
@TeleOp
public class DriveIntakeDemo extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this);
        robot.start();

        waitForStart();
        while (!isStopRequested()) {

            robot.drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            robot.intake.setPower(Math.max(gamepad1.left_trigger, gamepad1.right_trigger),
                    gamepad1.left_trigger > gamepad1.right_trigger ? Intake.IntakeMode.INTAKE : Intake.IntakeMode.OUTTAKE);

            robot.drive.update(null);

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
