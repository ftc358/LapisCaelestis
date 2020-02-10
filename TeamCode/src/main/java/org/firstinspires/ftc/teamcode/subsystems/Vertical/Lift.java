package org.firstinspires.ftc.teamcode.subsystems.Vertical;

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
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.LIFT_CONTROLLER_PID;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.MAX_ACC;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.MAX_JERK;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftConstants.kV;

public class Lift implements Subsystem {

    ExpansionHubMotor encoderMotor;
    DcMotor passiveMotor;

    PIDFController liftController;
    private NanoClock clock = NanoClock.system();

    private double power;
    private MotionProfile motionProfile;
    private MotionState motionState;
    private double startTime;
    private double velocityError;
    private double positionError;

    private double currentPosition;
    private double targetPosition;
    private double currentVel;
    private double targetVel;

    public TelemetryData telemetryData;
    private RevBulkData hubData;

    public enum LiftMode {
        IDLE,
        MOVING
    }

    public LiftMode mode;

    public class TelemetryData {
        public double LiftCurrentPosition;
        public double LiftTargetPosition;
        public double LiftCurrentVel;
        public double LiftTargetVel;
        public double LiftVelocityError;
        public double LiftPositionError;
        public double LiftPower;
        public LiftMode LiftMode;
    }

    public Lift(HardwareMap map) {
        telemetryData = new TelemetryData();

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

    public void setPosition(double position) {
        if (position != targetPosition) {
            targetPosition = position;
            startTime = clock.seconds();
            motionProfile = generateLiftMotionProfile(new MotionState(currentPosition, currentVel, 0), new MotionState(targetPosition, 0, 0));
            liftController.setTargetPosition(position);
            mode = LiftMode.MOVING;
        }
    }

    public double getLiftPosition() {
        if (hubData == null) {
            return 0.0;
        }
        return encoderTicksToInches(hubData.getMotorCurrentPosition(encoderMotor));
//        return encoderTicksToInches(hubData.getMotorCurrentPosition(encoderMotor));
    }

    public double getLiftVelocity() {
        if (hubData == null) {
            return 0.0;
        }
        return encoderTicksToInches(hubData.getMotorVelocity(encoderMotor));
//        return encoderTicksToInches(hubData.getMotorVelocity(encoderMotor));
    }

    private void resetEncoder() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double elapsedTime() {
        return clock.seconds() - startTime;
    }

    public boolean isBusy() {
        return mode != LiftMode.IDLE;
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
        hubData = Robot.expansionHubAData;
        currentPosition = getLiftPosition();
        currentVel = getLiftVelocity();

        switch (mode) {
            case IDLE: {
                power = 0.0;
                targetVel = 0.0;
                liftController.reset();
                encoderMotor.setPower(power);
                passiveMotor.setPower(power);
                break;
            }
            case MOVING: {
                motionState = motionProfile.get(elapsedTime());
                targetVel = motionState.getV();

                double correction = liftController.update(currentPosition, targetVel, motionState.getA());
                double correctedVelocity = targetVel + correction;
                power = Kinematics.calculateMotorFeedforward(correctedVelocity, motionState.getA(), kV, kA, kStatic);

                double timeRemaining = motionProfile.duration() - elapsedTime();
                velocityError = targetVel - currentVel;
                positionError = targetPosition - currentPosition;

                if (timeRemaining < 0) {
                    mode = LiftMode.IDLE;
                    power = 0.0;
                }
                encoderMotor.setPower(power);
                passiveMotor.setPower(power);
                break;
            }
        }

        telemetryData.LiftCurrentPosition = currentPosition;
        telemetryData.LiftTargetPosition = targetPosition;
        telemetryData.LiftCurrentVel = currentVel;
        telemetryData.LiftTargetVel = targetVel;
        telemetryData.LiftVelocityError = velocityError;
        telemetryData.LiftPositionError = positionError;
        telemetryData.LiftPower = power;
        telemetryData.LiftMode = mode;
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
