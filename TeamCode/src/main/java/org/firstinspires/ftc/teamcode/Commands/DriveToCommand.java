package org.firstinspires.ftc.teamcode.Commands;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;

import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Localization.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motion.WaypointFollower;
import org.firstinspires.ftc.teamcode.Pathing.PathCoordinator;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;

import java.util.List;
import java.util.Set;
public class DriveToCommand implements Command {

    private final WaypointFollower follower;
    private final PathCoordinator coordinator;
    private final PoseEstimator estimator;
    private final double goalX;
    private final double goalY;
    private double finalHeading;
    private final boolean hasHeadingOverride;

    public DriveToCommand (WaypointFollower follower, PathCoordinator coordinator, PoseEstimator estimator, double goalX, double goalY){
        this.follower = follower;
        this.coordinator = coordinator;
        this.estimator = estimator;
        this.goalX = goalX;
        this.goalY = goalY;
        this.hasHeadingOverride = false;
    }

    public DriveToCommand (WaypointFollower follower, PathCoordinator coordinator, PoseEstimator estimator, double goalX, double goalY, double finalHeadingRadians){
        this.follower = follower;
        this.coordinator = coordinator;
        this.estimator = estimator;
        this.goalX = goalX;
        this.goalY = goalY;
        this.finalHeading = finalHeadingRadians;
        this.hasHeadingOverride = true;
    }

    @Override
    public void start() {
        Pose start = estimator.getPose();
        List<Waypoint> path = hasHeadingOverride
                ? coordinator.getPath(start.x, start.y, goalX, goalY, finalHeading)
                : coordinator.getPath(start.x, start.y, goalX, goalY);
        follower.follow(path);
    }

    @Override
    public void execute() {
        Pose p = estimator.getPose();
        follower.update(p.x, p.y, p.heading);
    }

    @Override
    public boolean done() {
        return follower.isFinished();
    }

    @Override
    public void end(EndCondition endCondition) {
        follower.stop();
    }

    @Override
    public Set<Object> requirements() {
        return Set.of(follower);
    }

    @Override
    public int priority() {
        return 0;
    }

    @Override
    public InterruptedBehavior interruptedBehavior() {
        return InterruptedBehavior.SUSPEND;
    }

    @Override
    public BlockedBehavior blockedBehavior() {
        return BlockedBehavior.CANCEL;
    }

    @Override
    public ConflictBehavior conflictBehavior() {
        return ConflictBehavior.OVERRIDE;
    }
}