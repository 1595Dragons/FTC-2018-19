package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Stephen Ogden on 10/31/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Land", group = "Official")
public class landAuto extends LinearOpMode {

    private config robot = new config(this.telemetry);

    public void runOpMode() {

        robot.ConfigureRobot(this.hardwareMap);

        int stage = 0;

        waitForStart();
        robot.climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {
            switch (stage) {
                case 0:
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                    robot.climber.setPower(1);
                    stage++;
                    break;
                case 1:
                    if (robot.isThere(robot.climber, 10)) {
                        robot.climber.setPower(0);
                        stage++;
                    }
                    break;
                case 2:
                    stop();
                    break;

            }
            robot.updateTelemetry();
        }

    }
}
