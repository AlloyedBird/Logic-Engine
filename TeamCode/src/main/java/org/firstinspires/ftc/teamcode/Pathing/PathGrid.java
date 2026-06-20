package org.firstinspires.ftc.teamcode.Pathing;

import java.util.List;
import java.util.ArrayList;

public class PathGrid {

    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double RESOLUTION = 0.5; // inches per cell
    public static final int GRID_SIZE = (int)(FIELD_SIZE_INCHES / RESOLUTION); // 288

    private final boolean[] walkable;
    private final int inflationRadius;
    public PathGrid(int inflationRadiusCells) {
        this.walkable = new boolean[GRID_SIZE * GRID_SIZE];
        this.inflationRadius = inflationRadiusCells;
        // everything walkable by default
        java.util.Arrays.fill(walkable, true);
    }


    public int toGrid(double inches) {
        return (int)(inches / RESOLUTION);
    }

    public double toInches(int cell) {
        return cell * RESOLUTION;
    }

    private int index(int x, int y) {
        return y * GRID_SIZE + x;
    }

    public boolean inBounds(int x, int y) {
        return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
    }

    public boolean isWalkable(int x, int y) {
        return inBounds(x, y) && walkable[index(x, y)];
    }


    private void markInflated(int cx, int cy, boolean value) {
        for (int dx = -inflationRadius; dx <= inflationRadius; dx++) {
            for (int dy = -inflationRadius; dy <= inflationRadius; dy++) {
                int nx = cx + dx;
                int ny = cy + dy;
                if (inBounds(nx, ny)) {
                    walkable[index(nx, ny)] = value;
                }
            }
        }
    }

    public void addStaticObstacles(List<double[]> pointsInches) {
        for (double[] p : pointsInches) {
            markInflated(toGrid(p[0]), toGrid(p[1]), false);
        }
    }

    public void updateDynamicObstacles(List<double[]> pointsInches) {
        java.util.Arrays.fill(walkable, true);
        addStaticObstacles(staticObstacleCache);
        for (double[] p : pointsInches) {
            markInflated(toGrid(p[0]), toGrid(p[1]), false);
        }
    }

    private List<double[]> staticObstacleCache = new ArrayList<>();

    public void addStaticObstacles(List<double[]> pointsInches, boolean cache) {
        if (cache) staticObstacleCache = new ArrayList<>(pointsInches);
        addStaticObstacles(pointsInches);
    }
}