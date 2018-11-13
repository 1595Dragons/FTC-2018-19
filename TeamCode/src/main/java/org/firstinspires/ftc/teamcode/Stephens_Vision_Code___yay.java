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
@Disabled
@TeleOp(name = "Stephens vision test code", group = "Test")
public class Stephens_Vision_Code___yay extends LinearOpMode {

    private config robot = new config(this.telemetry);

    @Override
    public void runOpMode() {

        robot.InitializeVision(this.hardwareMap, false);

        waitForStart();

        // Start tracking
        robot.StartTrackingVisionTargets();

        while (opModeIsActive()) {

            // Iterate through the image targets, and if one is visible, set the found vision target
            for (VuforiaTrackable Target : robot.VisionTargets) {
                if (((VuforiaTrackableDefaultListener) Target.getListener()).isVisible()) {
                    robot.target = Target.getName();
                }
            }

            // Update the telemetry to display critical information
            robot.updateTelemetry();

        }

        // Stop tracking
        robot.StopTrackingVisionTargets();
    }
}