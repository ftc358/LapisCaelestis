package org.firstinspires.ftc.teamcode.subsystems.Vertical;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.Map;

public class HorizontalFeeder implements Subsystem {

    CRServo horizontalServo;
    Servo frontGrabber;
    Servo backGrabber;

    private TelemetryData telemetryData;

    private double horizontalPower;
    private double frontGrabberPosition;
    private double backGrabberPosition;

    public enum GrabberState {
        OPEN,
        CLOSE
    }

    public GrabberState state;

    public class TelemetryData {
        public double horizontalPower;
        public double frontGrabberPosition;
        public double backGrabberPosition;
        public GrabberState grabberState;
    }

    public HorizontalFeeder(HardwareMap map) {
        telemetryData = new TelemetryData();
        horizontalServo = map.crservo.get("horizontalServo");
        // TODO: direction?
        frontGrabber = map.servo.get("frontGrabber");
        backGrabber = map.servo.get("backGrabber");
    }

    public void setHorizontalPower(double power) {
        horizontalPower = power;
    }

    public void setFrontGrabberPosition(double position) {
        frontGrabberPosition = position;
    }

    public void setBackGrabberPosition(double position) {
        backGrabberPosition = position;
    }

    public void setGrabberPostions(double position) {
        frontGrabberPosition = position;
        backGrabberPosition = position;
        // TODO: direction?
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        horizontalServo.setPower(horizontalPower);
        frontGrabber.setPosition(frontGrabberPosition);
        backGrabber.setPosition(backGrabberPosition);

        telemetryData.horizontalPower = horizontalPower;
        telemetryData.frontGrabberPosition = frontGrabberPosition;
        telemetryData.backGrabberPosition = backGrabberPosition;
        telemetryData.grabberState = state;
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
