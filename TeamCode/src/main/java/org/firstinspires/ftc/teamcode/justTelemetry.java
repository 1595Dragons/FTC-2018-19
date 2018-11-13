package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 11/13/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Telemetry", group = "Test")
public class justTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() {
        config robot = new config(this.telemetry);
        robot.ConfigureRobtHardware(this.hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.updateTelemetry();
        }
    }
}
