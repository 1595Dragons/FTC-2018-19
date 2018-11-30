package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 11/5/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Vision test", group = "Test")
public class VisionTest extends LinearOpMode {

    private RobotConfig robot = new RobotConfig(this.telemetry);

    @Override
    public void runOpMode() {

        robot.setupGoldDetector(this.hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            robot.updateTelemetry();

        }

        // Disable the detector
        robot.goldDetector.disable();

    }
}
