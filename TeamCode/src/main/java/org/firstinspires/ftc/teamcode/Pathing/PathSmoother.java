package org.firstinspires.ftc.teamcode.Pathing;

import java.util.ArrayList;
import java.util.List;

public class PathSmoother {

    private final PathGrid grid;

    public PathSmoother(PathGrid grid) {
        this.grid = grid;
    }

    public List<double[]> smooth(List<double[]> path) {
        if (path.size() <= 2) return path;

        List<double[]> smoothed = new ArrayList<>();
        smoothed.add(path.get(0));

        int current = 0;

        while (current < path.size() - 1) {
            int furthest = current + 1;

            // Find the furthest waypoint we have line of sight to
            for (int i = current + 2; i < path.size(); i++) {
                if (hasLineOfSight(path.get(current), path.get(i))) {
                    furthest = i;
                } else {
                    break;
                }
            }

            smoothed.add(path.get(furthest));
            current = furthest;
        }

        return smoothed;
    }

    private boolean hasLineOfSight(double[] a, double[] b) {
        int x0 = grid.toGrid(a[0]), y0 = grid.toGrid(a[1]);
        int x1 = grid.toGrid(b[0]), y1 = grid.toGrid(b[1]);

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        int x = x0, y = y0;

        while (true) {
            if (!grid.isWalkable(x, y)) return false;
            if (x == x1 && y == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }

        return true;
    }
}