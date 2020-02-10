package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachingServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.Map;

public class FoundationGrabber implements Subsystem {

    private Servo servo;
    private DigitalChannel foundationSwitch;

    private TelemetryData telemetryData;

    private double servoPosition;

    public boolean foundationSwitchEngaged = false;

    public enum GrabberPosition {
        STOWED,
        LOWERED
    }

    private class TelemetryData {
        public double servoPosition;
        public boolean foundationSwitchEngaged;
    }

    public FoundationGrabber(HardwareMap map) {
        telemetryData = new TelemetryData();
        servo = new CachingServo(map.servo.get("foundationGrabber"));
//        servo = map.servo.get("foundationGrabber");
        foundationSwitch = map.get(DigitalChannel.class, "foundationSwitch");
        foundationSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setServoPosition(double position) {
        servoPosition = position;
    }

    public void setGrabberPosition(GrabberPosition position) {
        servoPosition = position == GrabberPosition.STOWED ? 0 : 1;
    }

    @Override
    public Map<String, Object> update(Canvas fieldOverlay) {
        telemetryData.servoPosition = servoPosition;
        telemetryData.foundationSwitchEngaged = foundationSwitchEngaged;
        servo.setPosition(servoPosition);
        if (!foundationSwitch.getState()) {
            foundationSwitchEngaged = true;
        }
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
