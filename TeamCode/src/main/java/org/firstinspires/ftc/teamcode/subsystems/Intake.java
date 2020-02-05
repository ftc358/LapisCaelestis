package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CachingDcMotor;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.Map;

public class Intake implements Subsystem {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private double intakePower;

    public enum IntakeMode {
        INTAKE,
        OUTTAKE
    }

    private TelemetryData telemetryData;

    private class TelemetryData {
        public double leftMotorPower;
        public double rightMotorPower;
    }

    public Intake(HardwareMap map) {
        telemetryData = new TelemetryData();
        leftMotor = new CachingDcMotor(map.dcMotor.get("leftIntakeMotor"));
        rightMotor = new CachingDcMotor(map.dcMotor.get("rightIntakeMotor"));
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power, IntakeMode mode) {
        intakePower = mode == IntakeMode.INTAKE ? power : -power;
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        leftMotor.setPower(intakePower);
        rightMotor.setPower(intakePower);
        telemetryData.leftMotorPower = leftMotor.getPower();
        telemetryData.rightMotorPower = rightMotor.getPower();
        return TelemetryUtil.objectToMap(telemetryData);
    }
}