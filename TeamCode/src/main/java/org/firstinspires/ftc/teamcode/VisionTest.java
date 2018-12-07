package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Created by Stephen Ogden on 10/10/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name = "Vision test code", group = "Test")
public class VisionTest extends LinearOpMode {

    private Config robot = new Config(this);

    @Override
    public void runOpMode() {

        robot.setupGoldDetector();

        waitForStart();

        robot.goldDetector.enable();

        while (opModeIsActive()) {

            if (robot.goldDetector.isFound()) {
                telemetry.addData("Gold detector", "Found!");
                telemetry.addData("Location", String.format(Locale.US, "%s, %s", robot.goldDetector.getScreenPosition().x, robot.goldDetector.getScreenPosition().y));
            } else {
                telemetry.addData("Gold detector", "Still searching");
            }

            telemetry.update();

        }
        robot.goldDetector.disable();
    }
}