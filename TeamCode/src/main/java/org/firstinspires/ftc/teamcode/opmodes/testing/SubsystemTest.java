package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;

@Config
@TeleOp
public class SubsystemTest extends LinearOpMode {

    Lift lift;
    public static double position = 1000.0;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        lift.setPosition(position);
        while (opModeIsActive() && lift.isBusy()) {
            lift.update(null);
            telemetry.addData("currentPosition", lift.telemetryData.LiftCurrentPosition);
            telemetry.addData("targetPosition", lift.telemetryData.LiftTargetPosition);
            telemetry.addData("currentVel", lift.telemetryData.LiftCurrentVel);
            telemetry.addData("targetVel", lift.telemetryData.LiftTargetVel);
            telemetry.addData("velocityError", lift.telemetryData.LiftVelocityError);
            telemetry.addData("positionError", lift.telemetryData.LiftPositionError);
            telemetry.addData("power", lift.telemetryData.LiftPower);
            telemetry.addData("mode", lift.telemetryData.LiftMode);
            telemetry.update();
        }
    }
}
