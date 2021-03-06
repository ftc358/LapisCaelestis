package org.firstinspires.ftc.teamcode.subsystems.drive.mecanum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kV;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class SampleMecanumDriveBase extends MecanumDrive implements Subsystem {
    //    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.5, 0, 0.0098);
    public static PIDCoefficients TRANSLATIONAL_PID_X = new PIDCoefficients(5, 0, 0.2);
    public static PIDCoefficients TRANSLATIONAL_PID_Y = new PIDCoefficients(5, 0, 0.1);
    //    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.001, 0, 0.01);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0.2);


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private TelemetryData telemetryData;
    private FtcDashboard dashboard;
    private NanoClock clock;

    public Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    public DriveConstraints constraints;
    public TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    public SampleMecanumDriveBase() {
        super(kV, kA, kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetryData = new TelemetryData();

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID_X, TRANSLATIONAL_PID_Y, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        updateUntilIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        updateUntilIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public Map<String, Object> update(Canvas fieldOverlay) {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        telemetryData.mode = mode;
        telemetryData.x = currentPose.getX();
        telemetryData.y = currentPose.getY();
        telemetryData.heading = currentPose.getHeading();

        telemetryData.xError = lastError.getX();
        telemetryData.yError = lastError.getY();
        telemetryData.headingError = lastError.getHeading();

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                if (fieldOverlay != null) {
                    Trajectory trajectory = follower.getTrajectory();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke("4CAF50");
                    DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());

                    fieldOverlay.setStroke("#F44336");
                    double t = follower.elapsedTime();
                    DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                    fieldOverlay.setStroke("#3F51B5");
                    fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
                }

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }
                break;
            }
        }

//        //TODO: remove this
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.putAll(TelemetryUtil.objectToMap(telemetryData));
//        dashboard.sendTelemetryPacket(packet);

        return TelemetryUtil.objectToMap(telemetryData);
    }

    public void updateUntilIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update(null);
        }
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

    private class TelemetryData {
        public Mode mode;
        public double x;
        public double y;
        public double heading;
        public double xError;
        public double yError;
        public double headingError;
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());;
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
