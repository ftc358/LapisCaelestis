package org.firstinspires.ftc.teamcode.subsystems.drive.testing;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.mecanum.MecanumDriveREVOptimized;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class CameraTest extends LinearOpMode {

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    public static int xPos = 85;
    public static int yPos = 90;

    public static int stoneWidth = 62;
    public static int stoneHeight = 30;

    public int stonePosition;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera phoneCam;
    SamplePipeline pipeline;

    // comment out later
    Bitmap bmp = null;

    // comment out later
    Runnable submitImage = new Runnable() {
        @Override
        public void run() {
            dashboard.sendImage(bmp);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        pipeline = new SamplePipeline();
        phoneCam.setPipeline(pipeline);

        // comment out later
        ExecutorService networking = Executors.newSingleThreadExecutor();

        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (!isStopRequested()) {

            stonePosition = pipeline.getPosition();

            telemetry.addData("skystonePosition", stonePosition);
            telemetry.update();

            networking.submit(submitImage);
        }
    }

    protected class SamplePipeline extends OpenCvPipeline {

        private int skystonePosition;
        Scalar green = new Scalar(0, 0, 255);

        // defining stone detection zones
        Rect rectLeft = new Rect(yPos, xPos, stoneHeight, stoneWidth);
        Rect rectMiddle = new Rect(yPos, xPos + stoneWidth, stoneHeight, stoneWidth);
        Rect rectRight = new Rect(yPos, xPos + 2 * stoneWidth, stoneHeight, stoneWidth);

        @Override
        public Mat processFrame(Mat input) {

            Mat left = new Mat(input, rectLeft);
            Mat middle = new Mat(input, rectMiddle);
            Mat right = new Mat(input, rectRight);

            double leftValue = getValue(Core.mean(left));
            double middleValue = getValue(Core.mean(middle));
            double rightValue = getValue(Core.mean(right));

            // make an arrayList with the values of the 3 blocks for easier comparison
            ArrayList<Double> values = new ArrayList<>();
            values.add(leftValue);
            values.add(middleValue);
            values.add(rightValue);

            skystonePosition = values.indexOf(Collections.min(values));

            // drawing things on the screen so that things can be seen on screen
            Imgproc.rectangle(input, rectLeft, green, 2);
            Imgproc.rectangle(input, rectMiddle, green, 2);
            Imgproc.rectangle(input, rectRight, green, 2);

            // convert the image to a bitmap to be sent to the dashboard
            try {
                bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(input, bmp);
            } catch (CvException e) {
                Log.d("Bitmap Creation Failure", e.getMessage());
            }

            return input;
        }

        private int getPosition() {
            return skystonePosition;
        }

        private double getValue(Scalar scalar) {
            double max = 0;

            for (int i = 0; i < 3; i++) {
                if (max < scalar.val[i]) {
                    max = scalar.val[i];
                }
            }

            return max;
        }

    }
}