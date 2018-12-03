package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 11/13/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Just telemetry", group = "Test")
public class justTelemetry extends LinearOpMode {
    Config robot = new Config(this);
    @Override
    public void runOpMode() {
        robot.ConfigureRobtHardware(true);
        waitForStart();
        while (opModeIsActive()) {
            robot.updateTelemetry();
        }
    }
}
