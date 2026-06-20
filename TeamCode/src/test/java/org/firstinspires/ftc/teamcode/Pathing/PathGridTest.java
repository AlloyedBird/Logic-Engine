package org.firstinspires.ftc.teamcode.Pathing;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class PathGridTest {

    @Test
    public void toGridAndToInches_roundTripOnResolutionAlignedValues() {
        PathGrid grid = new PathGrid(0);

        for (double inches : new double[]{ 0, 0.5, 1.0, 10.5, 71.5, 143.5 }) {
            int cell = grid.toGrid(inches);
            assertEquals(inches, grid.toInches(cell), 1e-9);
        }
    }

    @Test
    public void freshGrid_everyInBoundsCellIsWalkable() {
        PathGrid grid = new PathGrid(0);

        assertTrue(grid.isWalkable(0, 0));
        assertTrue(grid.isWalkable(PathGrid.GRID_SIZE - 1, PathGrid.GRID_SIZE - 1));
        assertTrue(grid.isWalkable(100, 200));
    }

    @Test
    public void isWalkable_outOfBoundsCellsAreNotWalkable() {
        PathGrid grid = new PathGrid(0);

        assertFalse(grid.isWalkable(-1, 0));
        assertFalse(grid.isWalkable(0, -1));
        assertFalse(grid.isWalkable(PathGrid.GRID_SIZE, 0));
        assertFalse(grid.isWalkable(0, PathGrid.GRID_SIZE));
    }

    @Test
    public void inBounds_matchesGridSizeBoundary() {
        PathGrid grid = new PathGrid(0);

        assertTrue(grid.inBounds(0, 0));
        assertTrue(grid.inBounds(PathGrid.GRID_SIZE - 1, PathGrid.GRID_SIZE - 1));
        assertFalse(grid.inBounds(PathGrid.GRID_SIZE, 0));
        assertFalse(grid.inBounds(-1, 0));
    }

    @Test
    public void addStaticObstacles_inflationRadiusBlocksNeighboringCells() {
        PathGrid grid = new PathGrid(2);
        int cx = grid.toGrid(50);
        int cy = grid.toGrid(50);

        List<double[]> obstacle = Collections.singletonList(new double[]{ 50, 50 });
        grid.addStaticObstacles(obstacle, true);

        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                assertFalse("cell (" + (cx + dx) + "," + (cy + dy) + ") within inflation radius should be blocked",
                        grid.isWalkable(cx + dx, cy + dy));
            }
        }
        // Just outside the inflation radius should remain walkable.
        assertTrue(grid.isWalkable(cx + 3, cy));
        assertTrue(grid.isWalkable(cx, cy + 3));
    }

    @Test
    public void updateDynamicObstacles_clearsPreviousDynamicCellsButKeepsStatics() {
        PathGrid grid = new PathGrid(0);
        int staticCell = grid.toGrid(20);
        int dynamicCellA = grid.toGrid(40);
        int dynamicCellB = grid.toGrid(60);

        grid.addStaticObstacles(Collections.singletonList(new double[]{ 20, 20 }), true);
        grid.updateDynamicObstacles(Collections.singletonList(new double[]{ 40, 40 }));

        assertFalse("static obstacle should remain blocked", grid.isWalkable(staticCell, staticCell));
        assertFalse("dynamic obstacle A should be blocked", grid.isWalkable(dynamicCellA, dynamicCellA));
        assertTrue("cell never marked as an obstacle should be walkable", grid.isWalkable(dynamicCellB, dynamicCellB));

        // Move the dynamic obstacle elsewhere; the old dynamic cell should reopen.
        grid.updateDynamicObstacles(Collections.singletonList(new double[]{ 60, 60 }));

        assertFalse("static obstacle should still remain blocked", grid.isWalkable(staticCell, staticCell));
        assertTrue("old dynamic obstacle cell should be walkable again", grid.isWalkable(dynamicCellA, dynamicCellA));
        assertFalse("new dynamic obstacle cell should be blocked", grid.isWalkable(dynamicCellB, dynamicCellB));
    }

    @Test
    public void updateDynamicObstacles_withEmptyList_leavesOnlyStaticsBlocked() {
        PathGrid grid = new PathGrid(0);
        int staticCell = grid.toGrid(20);
        int dynamicCell = grid.toGrid(40);

        grid.addStaticObstacles(Collections.singletonList(new double[]{ 20, 20 }), true);
        grid.updateDynamicObstacles(Collections.singletonList(new double[]{ 40, 40 }));
        grid.updateDynamicObstacles(new ArrayList<double[]>());

        assertFalse(grid.isWalkable(staticCell, staticCell));
        assertTrue(grid.isWalkable(dynamicCell, dynamicCell));
    }
}
