package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizer.StandardTrackingWheelLocalizer;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class RobotDiagnostics extends OpMode {
    private Robot robot;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private DcMotor leftEncoder, rightEncoder, backEncoder;
    private RevBulkData expansionHubAData;
    private RevBulkData expansionHubBData;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();
    }

    @Override
    public void loop() {
        expansionHubAData = Robot.expansionHubAData;
        expansionHubBData = Robot.expansionHubAData;

        robot.drive.setDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));

        robot.drive.update(null);

        Pose2d poseEstimate = robot.drive.getPoseEstimate();
        StandardTrackingWheelLocalizer localizer = (StandardTrackingWheelLocalizer) robot.drive.getLocalizer();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("leftEncoder", localizer.getWheelPositions().get(0));
        telemetry.addData("rightEncoder", localizer.getWheelPositions().get(1));
        telemetry.addData("backEncoder", localizer.getWheelPositions().get(2));
        telemetry.update();
    }
}
