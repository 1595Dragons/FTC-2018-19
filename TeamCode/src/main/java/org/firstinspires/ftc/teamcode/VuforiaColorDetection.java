package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 10/29/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
public class VuforiaColorDetection extends LinearOpMode {

    config robot = new config(this.telemetry);

    public void runOpMode() {

        robot.InitializeVision(this.hardwareMap);

        waitForStart();

        while (opModeIsActive()) {


            robot.updateTelemetry();

        }
    }
}
