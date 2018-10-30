package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Stephen Ogden on 10/29/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Vuforia Cube detection", group = "Test")
public class VuforiaCubeDetection extends LinearOpMode {

    public void runOpMode() {

        telemetry.addData("Status", "Initializing vision systems. Please wait...");
        telemetry.update();

        // Get the camera monitor id for the app
        int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());

        // Create a variable for passing parameters, such as the key for vuforia, and what camera we want to use (Back vs Selfie camera)
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUgZTU3/////AAAAGaQ5yTo6EkZqvsH9Iel0EktQjXWAZUz3q3FPq22sUTrmsYCccs/mjYiflQBH2u7lofbTxe4BxTca9o2EOnNwA8dLGa/yL3cUgDGjeRfXuwZUCpIG6OEKhiPU5ntOpT2Nr5uVkT3vs2uRr7J6G7YoaGHLw2i1wGncRaw37rZyO03QRh0ZatdKIiK1ItuvJkP3qfUJwQwcpROwa+ZdDNQDbpU6WTL+kPZpnkgR8oLcu+Na1lWrbJ2ZTYG8eUjoIGowbVVGJgORHJazy6/7MbYH268h9ZC4vZ12ItyDK/GlPRTeQWdcZRlWfzAAFwNrjmdjWv9hMuOMoWxo2Y2Rw1Fwii4ohLyRmcQa/wAWY+AOEL14";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        // Create the pictures engine and pass in the provided parameters
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Status", "Loading cube from assets");
        telemetry.update();

        VuforiaTrackables cube = vuforia.loadTrackablesFromAsset("Cube_OT");

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        cube.activate();
        while (opModeIsActive()) {

            if (((VuforiaTrackableDefaultListener) (cube.get(0).getListener())).isVisible()) {
                telemetry.addData("Status", "FOUND");
            } else {
                telemetry.addData("Status", "Still looking");
            }

            telemetry.update();
        }
        cube.deactivate();
    }
}