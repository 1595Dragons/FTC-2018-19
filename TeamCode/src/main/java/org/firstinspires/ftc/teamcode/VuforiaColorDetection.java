package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 10/29/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Vuforia color detection", group = "Test")
@Disabled
public class VuforiaColorDetection extends LinearOpMode {

    private config robot = new config(this.telemetry);

    public void runOpMode() {

        robot.InitializeVision(this.hardwareMap);

        waitForStart();
        robot.StartTrackingVisionTargets();
        while (opModeIsActive()) {

            robot.updateTelemetry();

        }
        robot.StopTrackingVisionTargets();
    }
}