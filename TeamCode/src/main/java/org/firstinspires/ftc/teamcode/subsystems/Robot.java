package org.firstinspires.ftc.teamcode.subsystems;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.subsystems.Vertical.HorizontalFeeder;
import org.firstinspires.ftc.teamcode.subsystems.Vertical.LiftSimple;
import org.firstinspires.ftc.teamcode.subsystems.drive.mecanum.MecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {

    public static final String TAG = "Robot";

    public FtcDashboard dashboard;

    private ExpansionHubEx expansionHubA, expansionHubB;

    public static RevBulkData expansionHubAData, expansionHubBData;

    public MecanumDriveREVOptimized drive;
    public Intake intake;
    public FoundationGrabber foundationGrabber;
    public LiftSimple lift;
    public HorizontalFeeder horizontalFeeder;

    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private List<CountDownLatch> cycleLatches;
    private OpModeManagerImpl opModeManager;
    private ExecutorService subsystemUpdateExecutor, telemetryUpdateExecutor;
    private BlockingQueue<TelemetryPacket> telemetryPacketQueue;

    public interface Listener {
        void onPostUpdate();
    }

    private List<Listener> listeners;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            try {
//                double startTimestamp = System.nanoTime() / Math.pow(10, 9);
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        expansionHubAData = expansionHubA.getBulkInputData();
                        expansionHubBData = expansionHubB.getBulkInputData();
                        Map<String, Object> telemetry = subsystem.update(telemetryPacket.fieldOverlay());
                        telemetryPacket.putAll(telemetry);
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            if (!subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.add(subsystem);
                            }
                        }
                    }
                }
                for (Listener listener : listeners) {
                    listener.onPostUpdate();
                }
//                double postSubsystemUpdateTimestamp = System.nanoTime() / Math.pow(10, 9);
                while (telemetryPacketQueue.remainingCapacity() == 0) {
                    Thread.sleep(1);
                }
                telemetryPacketQueue.add(telemetryPacket);
                synchronized (cycleLatches) {
                    int i = 0;
                    while (i < cycleLatches.size()) {
                        CountDownLatch latch = cycleLatches.get(i);
                        latch.countDown();
                        if (latch.getCount() == 0) {
                            cycleLatches.remove(i);
                        } else {
                            i++;
                        }
                    }
                }
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    private Runnable telemetryUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                TelemetryPacket packet = telemetryPacketQueue.take();
                dashboard.sendTelemetryPacket(packet);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }
    };

    public Robot(OpMode opMode) {
        dashboard = FtcDashboard.getInstance();

        listeners = new ArrayList<>();

        expansionHubA = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub A");
        expansionHubB = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub B");

        subsystems = new ArrayList<>();

        try {
            intake = new Intake(opMode.hardwareMap);
            subsystems.add(intake);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping intake");
        }

        try {
            drive = new MecanumDriveREVOptimized(opMode.hardwareMap);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping drive");
        }

        try {
            foundationGrabber = new FoundationGrabber(opMode.hardwareMap);
            subsystems.add(foundationGrabber);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, e);
            Log.w(TAG, "skipping foundationGrabber");
        }

        try {
            lift = new LiftSimple(opMode.hardwareMap);
            subsystems.add(lift);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, e);
            Log.w(TAG, "skipping lift simple");
        }

        try {
            horizontalFeeder = new HorizontalFeeder(opMode.hardwareMap);
            subsystems.add(horizontalFeeder);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, e);
            Log.w(TAG, "skipping horizaontal feeder");
        }

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
        telemetryUpdateExecutor = ThreadPool.newSingleThreadExecutor("telemetry update");

        telemetryPacketQueue = new ArrayBlockingQueue<>(10);

        subsystemsWithProblems = new ArrayList<>();
        RobotLog.registerGlobalWarningSource(this);

        cycleLatches = new ArrayList<>();
    }

    public void addListener(Listener listener) {
        listeners.add(listener);
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            telemetryUpdateExecutor.submit(telemetryUpdateRunnable);
            started = true;
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        if (telemetryUpdateExecutor != null) {
            telemetryUpdateExecutor.shutdownNow();
            telemetryUpdateExecutor = null;
        }

        RobotLog.unregisterGlobalWarningSource(this);
    }

    public void waitForNextCycle() {
        CountDownLatch latch = new CountDownLatch(1);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void waitOneFullCycle() {
        CountDownLatch latch = new CountDownLatch(2);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }

    @Override
    public String getGlobalWarning() {
        List<String> warnings = new ArrayList<>();
        synchronized (subsystemsWithProblems) {
            for (Subsystem subsystem : subsystemsWithProblems) {
                warnings.add("Problem with " + subsystem.getClass().getSimpleName());
            }
        }
        return RobotLog.combineGlobalWarnings(warnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblems) {
            subsystemsWithProblems.clear();
        }
    }
}
