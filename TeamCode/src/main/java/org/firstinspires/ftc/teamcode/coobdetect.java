package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Size;

/**
 * Created by Stephen Ogden on 11/5/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous (name = "Test cube detector", group = "Test")
class coobdetect extends LinearOpMode {

    private config robot = new config(this.telemetry);

    @Override
    public void runOpMode() {

        robot.status("Setting up experimental vision system");

        GoldDetector detector = new GoldDetector();
        detector.setAdjustedSize(new Size(480, 270));

        detector.init(this.hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.COLOR_DEVIATION;

        robot.status("Ready!");

        waitForStart();
        detector.enable();
        while (opModeIsActive()) {

            if (detector.isFound()) {
                detector.disable();
                robot.status("Found gold!");
                sleep(1000);
                stop();
            }

            robot.status("Searching...");

        }

        detector = null;

    }
}
