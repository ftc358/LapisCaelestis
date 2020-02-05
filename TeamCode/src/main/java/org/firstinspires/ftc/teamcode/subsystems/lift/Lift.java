package org.firstinspires.ftc.teamcode.subsystems.lift;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
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
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.generateLiftMotionProfile;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftConstants.kV;

@Config
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
    private int currentHeight;
    private int targetHeight;
    private double lastError;

    private TelemetryData telemetryData;
    private RevBulkData hubBulkData;

    public enum LiftMode {
        IDLE,
        MOVING
    }

    public LiftMode mode;

    private class TelemetryData {
        public double encoderMotorPower;
        public double passiveMotor;
        public double targetVel;
        public double currentVel;
        public int liftCurrentHeight;
        public int liftTargetHeight;
    }

    public Lift(HardwareMap map, RevBulkData bulkData) {
        telemetryData = new TelemetryData();
        hubBulkData = bulkData;

        mode = LiftMode.IDLE;
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

    public boolean isFollowing() {
        double timeRemaining = motionProfile.duration() - clock.seconds();
        return timeRemaining > 0;
    }

    public void moveLiftTo(int position) {
        targetHeight = position;
        motionProfile = generateLiftMotionProfile(new MotionState(currentHeight, 0, 0), new MotionState(targetHeight, 0, 0));
        startTime = clock.seconds();
    }

    public int getLiftPosition() {
        if (hubBulkData == null) {
            return 0;
        }
        return hubBulkData.getMotorCurrentPosition(encoderMotor);
    }

    public double getLiftVelocity() {
        if (hubBulkData == null) {
            return 0.0;
        }
        return hubBulkData.getMotorVelocity(encoderMotor);
    }

    private void resetEncoder() {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //TODO: limit switches?

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        currentHeight = getLiftPosition();

        switch (mode) {
            case IDLE: {
                break;
            }
            case MOVING: {
                MotionState state = motionProfile.get(clock.seconds() - startTime);
                currentVel = getLiftVelocity();
                targetVel = state.getV();
                lastError = targetVel - currentVel;
                double correction = liftController.update(currentHeight, state.getV(), state.getV());
                double correctedVelocity = targetVel + correction;
                power = Kinematics.calculateMotorFeedforward(correctedVelocity, state.getA(), kV, kA, kStatic);

                if (!isFollowing()) {
                    mode = LiftMode.IDLE;
                    power = 0;
                }
                encoderMotor.setPower(power);
                passiveMotor.setPower(power);
                break;
            }
        }

        telemetryData.currentVel = currentVel;
        telemetryData.targetVel = targetVel;
        telemetryData.liftCurrentHeight = currentHeight;
        telemetryData.liftTargetHeight = targetHeight;
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
