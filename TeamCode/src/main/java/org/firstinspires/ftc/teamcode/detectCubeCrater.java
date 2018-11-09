package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Stephen Ogden on 11/6/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@Autonomous(name = "Land and detect cube (crater)", group = "Official")
@Disabled // For now disable the motor until camera mount is decided
public class detectCubeCrater extends LinearOpMode {

    private RobotConfig robot = new RobotConfig(this.telemetry);

    private int stage = 0;

    @Override
    public void runOpMode() {

        robot.configureRobot(this.hardwareMap);
        robot.resetMotors(robot.climber, robot.left1, robot.left2, robot.right1, robot.right2);

        ElapsedTime time = new ElapsedTime();

        waitForStart();
        while (opModeIsActive()) {

            if (stage == 0) {
                robot.climber.setTargetPosition(robot.maxClimberPos);
                robot.climber.setPower(1);
                if (robot.isThere(5, robot.climber)) {
                    robot.climber.setPower(0);
                    stage++;
                }
            } else if (stage == 1) {
                robot.driveDistance(MecanumDriveDirection.RIGHT, 3, 1);
                if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                    robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    stage++;
                }
            } else if (stage == 2) {
                robot.climber.setTargetPosition(robot.minClimberPos);
                robot.climber.setPower(1);
                if (robot.isThere(5, robot.climber)) {
                    robot.climber.setPower(0);
                    stage++;
                }
            } else if (stage == 3) {
                robot.driveDistance(MecanumDriveDirection.FORWARD, 12, 1);
                if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                    robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    stage++;
                }
            } else if (stage == 4) {
                robot.driveDistance(MecanumDriveDirection.LEFT, 3, 1);
                if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                    robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    stage++;
                }
            } else if (stage == 5) {
                time.reset();
                robot.setupGoldDetector(this.hardwareMap);
                stage++;
            } else if (stage == 6) {
                if (time.seconds() > 3) {
                    stage++;
                }
            } else if (stage == 7) {
                if (robot.goldDetector.isFound()) {
                    stage = 16;
                } else {
                    stage++;
                }
            } else if (stage == 8) {
                robot.driveDistance(MecanumDriveDirection.LEFT, 16, 1);
                if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                    robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    stage++;
                }
            } else if (stage == 9) {
                time.reset();
                stage++;
            } else if (stage == 10) {
                if (time.seconds() > 3) {
                    stage++;
                }
            } else if (stage == 11) {
                if (robot.goldDetector.isFound()) {
                    stage = 16;
                } else {
                    stage++;
                }
            } else if (stage == 12) {
                robot.driveDistance(MecanumDriveDirection.RIGHT, 36, 1);
                if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                    robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    stage++;
                }
            } else if (stage == 13) {
                time.reset();
                stage++;
                if (stage == 14) {
                    if (time.seconds() > 3) {
                        stage++;
                    }
                }
            } else if (stage == 15) {
                if (robot.goldDetector.isFound()) {
                    stage = 16;
                } else {
                    stage++;
                }
            } else if (stage == 16) {
                robot.driveDistance(MecanumDriveDirection.FORWARD, 12, 1);
                if (robot.isThere(10, robot.right1, robot.left1, robot.left2, robot.right2)) {
                    robot.resetMotors(robot.left2, robot.left1, robot.right1, robot.right2);
                    stage++;
                }
            } else if (stage == 17) {
                robot.goldDetector.disable();
                stop();
            }

            // TODO: Gets stuck after stage 1. Alsays on "Ready"

            telemetry.addData("Stage", stage);
            telemetry.addLine();
            robot.updateTelemetry();

            idle();

        }

    }
}
