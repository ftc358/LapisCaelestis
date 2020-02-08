package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

public abstract class AutoOpmode extends LinearOpMode {

    protected Robot robot;

    // CV

    protected OpenCvCamera phoneCam;
    protected SkystonePipeline skystonePipeline;

    // consts

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    public static int xPos = 85;
    public static int yPos = 90;

    public static int stoneWidth = 30;
    public static int stoneHeight = 62;


    // states

    protected Pose2d initialPose;

    public enum StonePosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public StonePosition stonePosition;

    protected abstract void setup();

    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.start();

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        phoneCam.openCameraDevice();

        skystonePipeline = new SkystonePipeline();
        phoneCam.setPipeline(skystonePipeline);
        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        setup();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        run();
    }

    protected class SkystonePipeline extends OpenCvPipeline {

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

            return input;
        }

        public StonePosition getPosition() {
            switch (skystonePosition) {
                case 0: {
                    return StonePosition.LEFT;
                }
                case 1: {
                    return StonePosition.MIDDLE;
                }
                case 2: {
                    return StonePosition.MIDDLE.RIGHT;
                }
                default: {
                    return StonePosition.LEFT;
                }
            }
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
