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

    public void bakeStaticObstacles(List<double[]> pointsInches) {
        grid.addStaticObstacles(pointsInches, true);
    }

    public void updateDynamicObstacles(List<double[]> pointsInches) {
        grid.updateDynamicObstacles(pointsInches);
    }

    public List<Waypoint> getPath(double startX, double startY, double goalX, double goalY) {
        List<double[]> raw      = planner.findPath(startX, startY, goalX, goalY);
        List<double[]> smoothed = smoother.smooth(raw);
        return annotateHeadings(smoothed);
    }

    public List<Waypoint> getPath(double startX, double startY, double goalX, double goalY,
                                  double finalHeadingRadians) {
        List<Waypoint> path = getPath(startX, startY, goalX, goalY);
        if (path.isEmpty()) return path;

        Waypoint last = path.get(path.size() - 1);
        path.set(path.size() - 1, new Waypoint(last.x, last.y, finalHeadingRadians, true));
        return path;
    }

    private List<Waypoint> annotateHeadings(List<double[]> points) {
        if (points.isEmpty()) return Collections.emptyList();

        List<Waypoint> waypoints = new ArrayList<>();

        for (int i = 0; i < points.size(); i++) {
            double x = points.get(i)[0];
            double y = points.get(i)[1];
            double heading;

            if (i < points.size() - 1) {
                double dx = points.get(i + 1)[0] - x;
                double dy = points.get(i + 1)[1] - y;
                heading = Math.atan2(dy, dx);
            } else if (i > 0) {
                heading = waypoints.get(i - 1).heading;
            } else {
                heading = 0.0;
            }

            waypoints.add(new Waypoint(x, y, heading));
        }

        return waypoints;
    }
}