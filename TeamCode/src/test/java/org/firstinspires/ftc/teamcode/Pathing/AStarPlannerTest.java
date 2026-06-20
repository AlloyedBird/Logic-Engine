package org.firstinspires.ftc.teamcode.Pathing;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class AStarPlannerTest {

    private static final double EPS = 1e-6;

    private static double dist(double[] a, double[] b) {
        double dx = b[0] - a[0];
        double dy = b[1] - a[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Sum of consecutive segment lengths; for an optimal grid path this equals cost * RESOLUTION. */
    private static double pathLength(List<double[]> path) {
        double total = 0;
        for (int i = 1; i < path.size(); i++) {
            total += dist(path.get(i - 1), path.get(i));
        }
        return total;
    }

    private static double octileCost(int dx, int dy) {
        dx = Math.abs(dx);
        dy = Math.abs(dy);
        double straight = 1.0;
        double diagonal = Math.sqrt(2);
        return straight * (dx + dy) + (diagonal - 2 * straight) * Math.min(dx, dy);
    }

    @Test
    public void findsPath_endpointsSnapToStartAndGoal() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> path = planner.findPath(10, 10, 20, 20);

        assertTrue(!path.isEmpty());
        double[] first = path.get(0);
        double[] last = path.get(path.size() - 1);
        assertEquals(grid.toInches(grid.toGrid(10)), first[0], EPS);
        assertEquals(grid.toInches(grid.toGrid(10)), first[1], EPS);
        assertEquals(grid.toInches(grid.toGrid(20)), last[0], EPS);
        assertEquals(grid.toInches(grid.toGrid(20)), last[1], EPS);
    }

    @Test
    public void path_isContiguous_everyStepIsASingleGridMove() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> path = planner.findPath(5, 5, 40, 15);
        assertTrue(path.size() > 1);

        double res = PathGrid.RESOLUTION;
        for (int i = 1; i < path.size(); i++) {
            double dx = Math.round((path.get(i)[0] - path.get(i - 1)[0]) / res);
            double dy = Math.round((path.get(i)[1] - path.get(i - 1)[1]) / res);
            // Each hop must be exactly one of the 8 grid neighbor directions -
            // this is what verifies the forward/backward chains were joined correctly.
            assertTrue("step was not a unit grid move: (" + dx + "," + dy + ")",
                    Math.abs(dx) <= 1 && Math.abs(dy) <= 1 && (dx != 0 || dy != 0));
        }
    }

    @Test
    public void startEqualsGoal_returnsSinglePointPath() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> path = planner.findPath(12, 12, 12, 12);

        assertEquals(1, path.size());
        assertEquals(grid.toInches(grid.toGrid(12)), path.get(0)[0], EPS);
        assertEquals(grid.toInches(grid.toGrid(12)), path.get(0)[1], EPS);
    }

    @Test
    public void openGrid_diagonalPath_costMatchesOctileDistanceExactly() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> path = planner.findPath(0, 0, 10, 10);

        int dxCells = grid.toGrid(10) - grid.toGrid(0);
        int dyCells = grid.toGrid(10) - grid.toGrid(0);
        double expectedLength = octileCost(dxCells, dyCells) * PathGrid.RESOLUTION;

        assertEquals(expectedLength, pathLength(path), EPS);
    }

    @Test
    public void openGrid_mixedPath_costMatchesOctileDistance() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> path = planner.findPath(0, 0, 15, 5);

        int dxCells = grid.toGrid(15) - grid.toGrid(0);
        int dyCells = grid.toGrid(5) - grid.toGrid(0);
        double expectedLength = octileCost(dxCells, dyCells) * PathGrid.RESOLUTION;

        assertEquals(expectedLength, pathLength(path), EPS);
    }

    @Test
    public void path_isSymmetric_reversingStartAndGoalGivesSameCost() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> forward = planner.findPath(8, 30, 50, 4);
        List<double[]> backward = planner.findPath(50, 4, 8, 30);

        assertEquals(pathLength(forward), pathLength(backward), EPS);
    }

    @Test
    public void unreachableGoal_fullWallWithNoGap_returnsEmptyList() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        double wallXInches = 72;
        List<double[]> wall = new ArrayList<>();
        for (int gy = 0; gy < PathGrid.GRID_SIZE; gy++) {
            wall.add(new double[]{ wallXInches, grid.toInches(gy) });
        }
        grid.addStaticObstacles(wall, true);

        List<double[]> path = planner.findPath(20, 50, 120, 50);

        assertTrue(path.isEmpty());
    }

    @Test
    public void reachableGoal_wallWithGap_routesThroughGapAndAvoidsWall() {
        PathGrid grid = new PathGrid(0);
        AStarPlanner planner = new AStarPlanner(grid);

        double wallXInches = 72;
        int gapRow = 100; // leave this single row open, away from the endpoints' rows
        List<double[]> wall = new ArrayList<>();
        for (int gy = 0; gy < PathGrid.GRID_SIZE; gy++) {
            if (gy == gapRow) continue;
            wall.add(new double[]{ wallXInches, grid.toInches(gy) });
        }
        grid.addStaticObstacles(wall, true);

        // Start/goal rows are far from the gap row, forcing a vertical detour to reach it.
        List<double[]> path = planner.findPath(20, 10, 120, 90);

        assertTrue(!path.isEmpty());
        for (double[] point : path) {
            int gx = grid.toGrid(point[0]);
            int gy = grid.toGrid(point[1]);
            assertTrue("path passed through a blocked cell at (" + gx + "," + gy + ")",
                    grid.isWalkable(gx, gy));
        }
    }

    @Test
    public void everyWaypoint_inAnObstacleCourse_isWalkable() {
        PathGrid grid = new PathGrid(1);
        AStarPlanner planner = new AStarPlanner(grid);

        List<double[]> obstacles = new ArrayList<>();
        obstacles.add(new double[]{ 30, 30 });
        obstacles.add(new double[]{ 31, 32 });
        obstacles.add(new double[]{ 60, 60 });
        grid.addStaticObstacles(obstacles, true);

        List<double[]> path = planner.findPath(10, 10, 90, 90);

        assertTrue(!path.isEmpty());
        for (double[] point : path) {
            assertTrue(grid.isWalkable(grid.toGrid(point[0]), grid.toGrid(point[1])));
        }
    }
}
