package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class limitSwitch extends LinearOpMode {

    private DigitalChannel switch1;
    private DigitalChannel switch2;

    @Override
    public void runOpMode() throws InterruptedException {
                                                             // GND connected to COM
        switch1 = hardwareMap.digitalChannel.get("switch1"); // Channel A; normally false; connected to NC
        switch2 = hardwareMap.digitalChannel.get("switch2"); // Channel B; normally true;  connected to NO

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            telemetry.addData("Channel A", switch1.getState());
            telemetry.addData("Channel B", switch2.getState());
            telemetry.update();

        }
    }
}
