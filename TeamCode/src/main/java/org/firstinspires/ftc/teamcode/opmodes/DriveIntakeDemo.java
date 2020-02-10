package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.TRACK_WIDTH;

@Config
@TeleOp
public class DriveIntakeDemo extends LinearOpMode {
    private Robot robot;

    public static DriveConstraints TELEOP_CONSTRAINTS;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(this);
        robot.start();

        TELEOP_CONSTRAINTS = new DriveConstraints(
                50.0, 70.0, 0,
                Math.toRadians(270.0), Math.toRadians(270.0), 0.0
        );


        DriveConstraints constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);

        robot.drive.constraints = constraints;

        waitForStart();
        while (!isStopRequested()) {
            robot.drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            robot.intake.setPower(Math.max(gamepad1.left_trigger, gamepad1.right_trigger),
                    gamepad1.left_trigger > gamepad1.right_trigger ? Intake.IntakeMode.INTAKE : Intake.IntakeMode.OUTTAKE);

            robot.foundationGrabber.setServoPosition(gamepad1.right_trigger);

            robot.lift.setPower(gamepad2.left_stick_y);
            robot.horizontalFeeder.setHorizontalPower(gamepad2.right_stick_y);
            robot.horizontalFeeder.setFrontGrabberPosition(0.5 - gamepad2.left_trigger);
            robot.horizontalFeeder.setBackGrabberPosition(0.9 + gamepad2.right_trigger);
        }
    }
}
