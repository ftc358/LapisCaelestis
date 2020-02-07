package org.firstinspires.ftc.teamcode.subsystems.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 2400;
    public static double WHEEL_RADIUS = 0.94488; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

//    public static double LATERAL_DISTANCE = 16.75; // in; distance between the left and right wheels
    // we are not using it
//    public static double FORWARD_OFFSET = 8.00; // in; offset of the lateral wheel

    private ExpansionHubEx hub;

    private DcMotor leftEncoder, rightEncoder, backEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, 7.835, 0), // left
                new Pose2d(0, -7.835, 0), // right
                new Pose2d(-7.835, 1.378, Math.toRadians(90)) // back
        ));

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub B");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder-rightIntake");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder-leftIntake");
        backEncoder = hardwareMap.dcMotor.get("backEncoder");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        backEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(leftEncoder)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(rightEncoder)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(backEncoder)));

        return wheelPositions;

//        return Arrays.asList(
//                lateralEncoderTicksToInches(leftEncoder.getCurrentPosition()),
//                lateralEncoderTicksToInches(rightEncoder.getCurrentPosition()),
//                frontEncoderTicksToInches(backEncoder.getCurrentPosition())
//        );
    }
}
