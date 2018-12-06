package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 10/10/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
//@Disabled
@TeleOp(name = "Vision test code", group = "Test")
public class VisionTest extends LinearOpMode {

    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        robot.setupGoldDetector();

        waitForStart();

        robot.goldDetector.enable();

        while (opModeIsActive()) {

            robot.updateTelemetry();

        }
        robot.goldDetector.disable();
    }
}