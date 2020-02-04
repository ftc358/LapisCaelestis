package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.CachingDcMotor;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.getMotorVelocityF;

@Config
public class Lift implements Subsystem {

    ExpansionHubMotor encoderMotor;
    DcMotor passiveMotor;

    PIDFController liftController;

    private double liftPower;
    private int liftCurrentHeight;
    private int liftTargetHeight;

    private TelemetryData telemetryData;
    private RevBulkData hubBulkData;

    public static final PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0, 0.0, 0.0);
    public static final PIDCoefficients LIFT_CONTROLLER_PID = new PIDCoefficients(0.0, 0.0, 0.0);

    private class TelemetryData {
        public double encoderMotorVelocity;
        public double passiveMotorVelocity;
        public int liftCurrentHeight;
        public int liftTargetHeight;
    }

    public Lift(HardwareMap map, RevBulkData bulkData) {
        telemetryData = new TelemetryData();
        hubBulkData = bulkData;

        liftController = new PIDFController(LIFT_CONTROLLER_PID);
        liftController.setOutputBounds(0.0, 1.0);

        encoderMotor = map.get(ExpansionHubMotor.class, "liftEncoderMotor");
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.kP, MOTOR_VELO_PID.kI, MOTOR_VELO_PID.kD, getMotorVelocityF()
        ));

        passiveMotor = new CachingDcMotor(map.dcMotor.get("liftPassiveMotor"));
        passiveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: directions?
    }

    public void moveLiftTo(int position) {
        liftTargetHeight = position;
    }

    private void resetEncoder() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //TODO: limit switches?

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        liftCurrentHeight = hubBulkData.getMotorCurrentPosition(encoderMotor);

        liftController.setTargetPosition(liftTargetHeight);
        liftPower = (liftCurrentHeight + liftController.update(liftCurrentHeight)) / liftTargetHeight;
        //TODO: motion profiling?
        //TODO: test--would it reach quiescence?
        //TODO: test--would initial power be too high?
        encoderMotor.setPower(liftPower);
        passiveMotor.setPower(liftPower);

        telemetryData.encoderMotorVelocity = hubBulkData.getMotorVelocity(encoderMotor);
        telemetryData.passiveMotorVelocity = hubBulkData.getMotorVelocity(passiveMotor);
        telemetryData.liftCurrentHeight = liftCurrentHeight;
        telemetryData.liftTargetHeight = liftTargetHeight;
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
