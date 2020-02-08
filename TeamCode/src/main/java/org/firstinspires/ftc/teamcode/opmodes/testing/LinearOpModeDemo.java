package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@Config
@TeleOp
public class LinearOpModeDemo extends LinearOpMode {

    private Robot robot;
    private StickyGamepad stickyGamepad;

    public static double height = 1000.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);
        robot.start();

        stickyGamepad = new StickyGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");

        waitForStart();

//        robot.lift.setPosition(height);
//        robot.lift.waitForIdle();
    }
}
