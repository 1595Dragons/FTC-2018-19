package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by Stephen Ogden on 10/10/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
//@Disabled
@TeleOp(name = "Vision test code", group = "Test")
public class VisionTest extends LinearOpMode {

    private config robot = new config(this.telemetry);

    @Override
    public void runOpMode() {

        robot.setupGoldDetector(this.hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            robot.updateTelemetry();

        }
        robot.goldDetector.disable();
    }
}