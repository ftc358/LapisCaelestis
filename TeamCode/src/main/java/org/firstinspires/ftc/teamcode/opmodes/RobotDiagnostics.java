package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class RobotDiagnostics extends OpMode {
    private Robot robot;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private DcMotor leftEncoder, rightEncoder, backEncoder;
    private RevBulkData expansionHubAData;
    private RevBulkData expansionHubBData;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder-rightIntake");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder-leftIntake");
        backEncoder = hardwareMap.dcMotor.get("backEncoder");
    }

    @Override
    public void loop() {
        expansionHubAData = Robot.expansionHubAData;
        expansionHubBData = Robot.expansionHubAData;

        telemetry.addData("Hub A, leftFront", expansionHubAData.getMotorCurrentPosition(leftFront));
        telemetry.addData("Hub A, leftRear", expansionHubAData.getMotorCurrentPosition(leftRear));
        telemetry.addData("Hub A, rightFront", expansionHubAData.getMotorCurrentPosition(rightFront));
        telemetry.addData("Hub A, rightRear", expansionHubAData.getMotorCurrentPosition(rightRear));

        telemetry.addData("Hub B, leftEncoder-rightIntake", expansionHubBData.getMotorCurrentPosition(leftEncoder));
        telemetry.addData("Hub B, rightEncoder-leftIntake", expansionHubBData.getMotorCurrentPosition(rightEncoder));
        telemetry.addData("Hub B, backEncoder", expansionHubBData.getMotorCurrentPosition(backEncoder));
    }
}
