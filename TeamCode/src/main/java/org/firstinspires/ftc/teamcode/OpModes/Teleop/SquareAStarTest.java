package org.firstinspires.ftc.teamcode.OpModes.Teleop;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Commands.DriveToCommand;
import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Localization.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motion.PIDController;
import org.firstinspires.ftc.teamcode.Motion.WaypointFollower;
import org.firstinspires.ftc.teamcode.Pathing.PathCoordinator;
import org.firstinspires.ftc.teamcode.Vision.ShinyObjectDetector;

import java.util.ArrayList;

@TeleOp(name = "Pathing Test", group = "Test")
public class SquareAStarTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // --- Hardware ---
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // --- Subsystems ---
        PathCoordinator coordinator = new PathCoordinator(2);
        ShinyObjectDetector shinyDetector = new ShinyObjectDetector();
        coordinator.bakeStaticObstacles(new ArrayList<>());

        PoseEstimator estimator = new PoseEstimator(hardwareMap, new Pose(0, 0,0), shinyDetector);

        PIDController xPid       = new PIDController(0.1, 0, 0.01);
        PIDController yPid       = new PIDController(0.1, 0, 0.01);
        PIDController headingPid = new PIDController(0.1, 0, 0.01);

        WaypointFollower follower = new WaypointFollower(
                frontLeft, frontRight, backLeft, backRight,
                xPid, yPid, headingPid,
                coordinator, estimator
        );

        // --- Commands ---
        Command driveToCenter = new DriveToCommand(follower, coordinator, estimator, 72, 72);

        Command driveSquare = Groups.sequential(
                new DriveToCommand(follower, coordinator, estimator, 54, 54),
                new DriveToCommand(follower, coordinator, estimator, 90, 54),
                new DriveToCommand(follower, coordinator, estimator, 90, 90),
                new DriveToCommand(follower, coordinator, estimator, 54, 90),
                new DriveToCommand(follower, coordinator, estimator, 54, 54),
                new DriveToCommand(follower, coordinator, estimator, 72, 72)
        );

        Command fullTest = Groups.sequential(driveToCenter, driveSquare);

        boolean lastA = false;

        waitForStart();

        while (opModeIsActive()) {

            // --- Trigger test on A press ---
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                Scheduler.schedule(fullTest);
            }
            lastA = currentA;

            // --- Ivy tick ---
            Scheduler.execute();

            // --- Telemetry ---
            Pose pose = estimator.getPose();
            telemetry.addData("x",       pose.x);
            telemetry.addData("y",       pose.y);
            telemetry.addData("heading", Math.toDegrees(pose.heading));
            telemetry.update();
        }

        estimator.stop();
    }
}