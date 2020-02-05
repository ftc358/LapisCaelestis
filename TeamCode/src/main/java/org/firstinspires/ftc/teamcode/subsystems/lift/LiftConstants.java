package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;

@Config
public class LiftConstants {

    public static final PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0, 0.0, 0.0);
    public static final PIDCoefficients LIFT_CONTROLLER_PID = new PIDCoefficients(0.0, 0.0, 0.0);

    public static final double MAX_VEL = 0;
    public static final double MAX_ACC = 0;
    public static final double MAX_JERK = 0;

    public static double SPOOL_RADIUS = 0.311;
    public static double GEAR_RATIO = 42.0 / 48.0; // output (wheel) speed / input (motor) speed

    public MotionProfile generateLiftMotionProfile(MotionState start, MotionState end) {
        return MotionProfileGenerator.generateSimpleMotionProfile(start, end, MAX_VEL, MAX_ACC, MAX_JERK, false);
    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / 293;
        // 293: calculated & experimentally confirmed
    }
}
