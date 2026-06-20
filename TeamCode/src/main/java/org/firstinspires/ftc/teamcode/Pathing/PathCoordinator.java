package org.firstinspires.ftc.teamcode.Pathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PathCoordinator {

    private final PathGrid grid;
    private final AStarPlanner planner;
    private final PathSmoother smoother;

    public PathCoordinator(int inflationRadiusCells) {
        this.grid     = new PathGrid(inflationRadiusCells);
        this.planner  = new AStarPlanner(grid);
        this.smoother = new PathSmoother(grid);
    }

    // --- Setup ---

    /** Call once at init with all static field elements. */
    public void bakeStaticObstacles(List<double[]> pointsInches) {
        grid.addStaticObstacles(pointsInches, true);
    }

    /** Call every replan cycle with fresh robot/obstacle positions. */
    public void updateDynamicObstacles(List<double[]> pointsInches) {
        grid.updateDynamicObstacles(pointsInches);
    }

    // --- Path generation ---

    /**
     * Get a smoothed, heading-annotated path from start to goal.
     * All positions in inches. Heading computed from direction of travel.
     * Returns empty list if no path exists.
     */
    public List<Waypoint> getPath(double startX, double startY, double goalX, double goalY) {
        List<double[]> raw      = planner.findPath(startX, startY, goalX, goalY);
        List<double[]> smoothed = smoother.smooth(raw);
        return annotateHeadings(smoothed);
    }

    /**
     * Same as getPath but lets the caller override heading on the final waypoint.
     * Useful for docking, scoring, or any approach that needs a specific facing.
     */
    public List<Waypoint> getPath(double startX, double startY, double goalX, double goalY,
                                  double finalHeadingRadians) {
        List<Waypoint> path = getPath(startX, startY, goalX, goalY);
        if (path.isEmpty()) return path;

        // Replace last waypoint with heading override
        Waypoint last = path.get(path.size() - 1);
        path.set(path.size() - 1, new Waypoint(last.x, last.y, finalHeadingRadians, true));
        return path;
    }

    // --- Internal ---

    private List<Waypoint> annotateHeadings(List<double[]> points) {
        if (points.isEmpty()) return Collections.emptyList();

        List<Waypoint> waypoints = new ArrayList<>();

        for (int i = 0; i < points.size(); i++) {
            double x = points.get(i)[0];
            double y = points.get(i)[1];
            double heading;

            if (i < points.size() - 1) {
                // Face toward next waypoint
                double dx = points.get(i + 1)[0] - x;
                double dy = points.get(i + 1)[1] - y;
                heading = Math.atan2(dy, dx);
            } else if (i > 0) {
                // Last waypoint inherits previous heading
                heading = waypoints.get(i - 1).heading;
            } else {
                // Single-waypoint path (start and goal in the same cell) — no direction of travel available
                heading = 0.0;
            }

            waypoints.add(new Waypoint(x, y, heading));
        }

        return waypoints;
    }
}