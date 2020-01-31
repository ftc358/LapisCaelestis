package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@TeleOp
public class Demo extends OpMode {

    private Robot robot;
    private StickyGamepad stickyGamepad;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad = new StickyGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        stickyGamepad.update();
        robot.intake.setPower(Math.max(gamepad1.left_trigger, gamepad1.right_trigger),
                gamepad1.left_trigger > gamepad1.right_trigger ? Intake.IntakeMode.INTAKE : Intake.IntakeMode.OUTTAKE);
    }
}
