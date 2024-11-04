package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="PID-Testing", group="TeleOp")
public class pidfTesting extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 700 / 180.0;

    private DcMotorEx upDrive1;
    private DcMotorEx upDrive2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        upDrive1 = hardwareMap.get(DcMotorEx.class, "upDrive1");
        //upDrive2 = hardwareMap.get(DcMotorEx.class, "upDrive2");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int upDrive1Pos = upDrive1.getCurrentPosition();
        //int upDrive2Pos = upDrive2.getCurrentPosition();
        double pid = controller.calculate(upDrive1Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        upDrive1.setPower(power);

        telemetry.addData("pos", upDrive1Pos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
