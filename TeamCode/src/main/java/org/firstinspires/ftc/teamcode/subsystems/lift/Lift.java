package org.firstinspires.ftc.teamcode.subsystems.lift;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.CachingDcMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.LIFT_CONTROLLER_PID;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.MAX_ACC;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.MAX_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.kV;

public class Lift implements Subsystem {

    ExpansionHubMotor encoderMotor;
    DcMotor passiveMotor;

    PIDFController liftController;
    private NanoClock clock = NanoClock.system();

    private double power;
    private MotionProfile motionProfile;
    private MotionState motionState;
    private double startTime = 0.0;
    private double currentVel;
    private double targetVel;
    private double currentHeight;
    private double targetHeight = 0.0;
    private double lastError;

    private TelemetryData telemetryData;
    private RevBulkData hubData;

    public enum LiftMode {
        IDLE,
        MOVING
    }

    public LiftMode mode;

    private class TelemetryData {
        public double targetVel;
        public double currentVel;
        public double liftCurrentHeight;
        public double liftTargetHeight;
        public double lastError;
        public double correction;
        public double correctedVelocity;
        public double power;
        public LiftMode mode;
    }

    public Lift(HardwareMap map, RevBulkData bulkData) {
        telemetryData = new TelemetryData();
        hubData = bulkData;

        mode = LiftMode.IDLE;
        liftController = new PIDFController(LIFT_CONTROLLER_PID);
        liftController.setOutputBounds(0.0, 1.0);

        encoderMotor = map.get(ExpansionHubMotor.class, "liftEncoderMotor");
        encoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.kP, MOTOR_VELO_PID.kI, MOTOR_VELO_PID.kD, getMotorVelocityF()
        ));

        passiveMotor = new CachingDcMotor(map.dcMotor.get("liftPassiveMotor"));
        passiveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: directions?
    }

    public boolean isBusy() {
        return mode != LiftMode.IDLE;
    }

    public void setHeight(double height) {
        if (height != targetHeight) {
            targetHeight = height;
            mode = LiftMode.MOVING;
            startTime = clock.seconds();
            liftController.setTargetPosition(height);
        }
    }

    public double getLiftPosition() {
        if (hubData == null) {
            return 0;
        }
        // TODO: enable bulk data
//        return encoderTicksToInches(hubData.getMotorCurrentPosition(encoderMotor));
        return encoderTicksToInches(encoderMotor.getCurrentPosition());
    }

    public double getLiftVelocity() {
        if (hubData == null) {
            return 0.0;
        }
//        return hubData.getMotorVelocity(encoderMotor);
        return encoderTicksToInches(encoderMotor.getVelocity());
    }

    private void resetEncoder() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public MotionProfile generateLiftMotionProfile(MotionState start, MotionState end) {
        return MotionProfileGenerator.generateSimpleMotionProfile(start, end, MAX_VEL, MAX_ACC, MAX_JERK, false);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    //TODO: limit switches?

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        currentHeight = getLiftPosition();

        switch (mode) {
            case IDLE: {
                startTime = 0.0;
                power = 0;
                encoderMotor.setPower(power);
                passiveMotor.setPower(power);
                break;
            }
            case MOVING: {
                motionProfile = generateLiftMotionProfile(new MotionState(currentHeight, 0, 0), new MotionState(targetHeight, 0, 0));
                motionState = motionProfile.get(clock.seconds() - startTime);
                currentVel = getLiftVelocity();
                targetVel = motionState.getV();

                double error = targetVel - currentVel;
                double correction = liftController.update(currentHeight, motionState.getV(), motionState.getA());
                telemetryData.correction = correction;
                lastError = error;
                double correctedVelocity = targetVel + correction;
                telemetryData.correctedVelocity = correctedVelocity;
                power = Kinematics.calculateMotorFeedforward(correctedVelocity, motionState.getA(), kV, kA, kStatic);
                telemetryData.power = power;
                double timeRemaining = motionProfile.duration() - (clock.seconds() - startTime);

                if (timeRemaining < 0) {
                    mode = LiftMode.IDLE;
                    targetVel = 0.0;
                    power = 0;
                }
                encoderMotor.setPower(power);
                passiveMotor.setPower(power);
                break;
            }
        }

        telemetryData.currentVel = getLiftVelocity();
        telemetryData.targetVel = targetVel;
        telemetryData.liftCurrentHeight = currentHeight;
        telemetryData.liftTargetHeight = targetHeight;
        telemetryData.lastError = lastError;
        telemetryData.mode = mode;
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
