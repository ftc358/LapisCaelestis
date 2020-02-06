package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

@Config
public class LiftConstants {

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(5.0, 0.0, 0.0);
    public static PIDCoefficients LIFT_CONTROLLER_PID = new PIDCoefficients(5.0, 0.0, 0.0);

    public static double MAX_ADMISSIBLE_ERROR = 0.5;

    public static final double MAX_VEL = 10.0;
    public static final double MAX_ACC = 20.0;
    public static final double MAX_JERK = 0;

    public static final double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public static final double kA = 0;
    public static final double kStatic = 0;

    public static double SPOOL_RADIUS = 0.311;
    public static double GEAR_RATIO = 42.0 / 48.0; // output (wheel) speed / input (motor) speed

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / 293.0;
        // 293: calculated & experimentally confirmed
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
//        return MOTOR_CONFIG.getMaxRPM() *
//                (RUN_USING_ENCODER ? MOTOR_CONFIG.getAchieveableMaxRPMFraction() : 1.0);
        // internal gear ratio: 304/29
        // 6000: http://www.revrobotics.com/rev-41-1301/
        // default AchieveableMaxRPMFraction: 0.85
//        return 6000.0 / (304.0 / 29.0) * 0.85;
        return 486.0;
    }
}
