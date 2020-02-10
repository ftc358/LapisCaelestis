package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SingleCrServoTest extends LinearOpMode {

    CRServo crServo;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        crServo = hardwareMap.crservo.get("crServo");

        waitForStart();

        while (!isStopRequested()) {
            crServo.setPower(gamepad1.left_stick_y);
            telemetry.addData("power", crServo.getPower());
            telemetry.update();
        }
    }
}
