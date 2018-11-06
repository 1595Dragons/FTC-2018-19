package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Stephen Ogden on 11/6/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Land and detect cube (crater)", group = "Official")
@Disabled // For now disable the motor until camera mount is decided
public class detectCubeCrater extends LinearOpMode {

    private config robot = new config(this.telemetry);

    @Override
    public void runOpMode() {

        robot.ConfigureRobot(this.hardwareMap);
        robot.setupGoldDetector(this.hardwareMap);

        int stage = 0, foundTimes = 0;

        waitForStart();
        robot.resetMotors(robot.climber, robot.left1, robot.left2, robot.right1, robot.right2);
        while (opModeIsActive()) {

            switch (stage) {
                case 0:
                    robot.climber.setTargetPosition(robot.maxClimberPos);
                    robot.climber.setPower(1);
                    if (robot.isThere(5, robot.climber)) {
                        stage++;
                    }
                    break;
                case 1:
                    // TODO: Turn to get camera facing objects
                    // For now, just pass
                    stage++;
                    break;
                case 2:
                    robot.goldDetector.enable();
                    if (robot.goldDetector.isFound()) {
                        foundTimes++;
                        if (foundTimes >= 5) {
                            robot.goldDetector.disable();
                            robot.resetMotors(robot.left1, robot.right1, robot.left2, robot.right2);
                            stage++;
                        }
                    }
                    break;
                case 3:
                    robot.driveDistance(MecanumDriveDirection.FORWARD, 24, 1);
                    if (robot.isThere(15, robot.left1) || robot.isThere(15, robot.right1) || robot.isThere(15, robot.left2) || robot.isThere(15, robot.right2)) {
                        robot.resetMotors(robot.left1, robot.right1, robot.left2, robot.right2);
                        stage++;
                    }
                    break;
                case 4:
                    stop();
                    break;
            }

            robot.updateTelemetry();

        }

    }
}
