package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachingDcMotor;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Map;

public class Lift implements Subsystem {

    ExpansionHubMotor encoderMotor;
    DcMotor passiveMotor;

    PIDFController liftController;

    private double liftPower;
    private int liftCurrentHeight;
    private int liftTargetHeight;

    private TelemetryData telemetryData;
    private RevBulkData hubBulkData;

    private class TelemetryData {
        public double encoderMotorVelocity;
        public double passiveMotorVelocity;
        public int liftCurrentHeight;
        public int liftTargetHeight;
    }

    public Lift(HardwareMap map, RevBulkData bulkData) {
        telemetryData = new TelemetryData();
        liftController = new PIDFController(new PIDCoefficients(5.0, 0.0, 0.0));
        liftController.setOutputBounds(0.0, 1.0);
        encoderMotor = map.get(ExpansionHubMotor.class, "liftEncoderMotor");
        passiveMotor = new CachingDcMotor(map.dcMotor.get("liftPassiveMotor"));
        //TODO: directions?
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hubBulkData = bulkData;
    }

//    private void moveLift(double power) {
//        liftController.setTargetPosition(power);
//        double correction = liftController.update(liftCurrentPower);
//    }

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
