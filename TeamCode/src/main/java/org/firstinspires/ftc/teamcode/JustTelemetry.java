package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Stephen Ogden on 10/30/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Just Telemetry", group = "Test")
public class JustTelemetry extends LinearOpMode {
    private RobotConfig robot = new RobotConfig(this.telemetry);
    public void runOpMode() {
        robot.configureRobot(this.hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.updateTelemetry();
        }
    }
}
