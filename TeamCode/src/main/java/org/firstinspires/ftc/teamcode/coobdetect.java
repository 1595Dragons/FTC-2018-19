package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Size;

import java.util.Locale;

/**
 * Created by Stephen Ogden on 11/5/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Test cube detector", group = "Test")
public class coobdetect extends LinearOpMode {

    private config robot = new config(this.telemetry);

    @Override
    public void runOpMode() {

        robot.status("Setting up experimental vision system");

        // Create a new detector object
        GoldDetector detector = new GoldDetector();

        // Set the size of the camera
        detector.setAdjustedSize(new Size(270, 480));

        // Init the detector (try to use the defaults)
        detector.init(this.hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Apply the score based on color
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.COLOR_DEVIATION;

        robot.status("Ready!");

        waitForStart();
        // Start looking for the cube
        detector.enable();
        while (opModeIsActive()) {

            // If the cube is found, let the user know
            if (detector.isFound()) {
                robot.status(String.format(Locale.US, "Found gold at %f, %f!", detector.getScreenPosition().x, detector.getScreenPosition().y));
            } else {

                // If its not found just continue searching
                robot.status("Searching...");

            }

        }

        // Disable the detector
        detector.disable();

    }
}
