package org.firstinspires.ftc.teamcode.Pathing;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

public class  AStarPlanner {

    private static final double STRAIGHT_COST = 1.0;
    private static final double DIAGONAL_COST = Math.sqrt(2);

    private static final int[][] NEIGHBORS = {
            { 1,  0}, {-1,  0}, { 0,  1}, { 0, -1}, // cardinal
            { 1,  1}, { 1, -1}, {-1,  1}, {-1, -1}  // diagonal
    };

    private static class Node implements Comparable<Node> {
        int x, y;
        double g, f;
        Node parent;

        Node(int x, int y, double g, double f, Node parent) {
            this.x = x;
            this.y = y;
            this.g = g;
            this.f = f;
            this.parent = parent;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.f, other.f);
        }
    }

    private final PathGrid grid;

    public AStarPlanner(PathGrid grid) {
        this.grid = grid;
    }

    public List<double[]> findPath(double startX, double startY, double goalX, double goalY) {
        int sx = grid.toGrid(startX), sy = grid.toGrid(startY);
        int gx = grid.toGrid(goalX), gy = grid.toGrid(goalY);

        PriorityQueue<Node> openF = new PriorityQueue<>();
        PriorityQueue<Node> openB = new PriorityQueue<>();

        HashMap<Integer, Double> bestGF = new HashMap<>();
        HashMap<Integer, Double> bestGB = new HashMap<>();

        HashMap<Integer, Node> parentF = new HashMap<>();
        HashMap<Integer, Node> parentB = new HashMap<>();

        double bestCost = Double.MAX_VALUE;
        int meetingKey = -1;

        Node startNode = new Node(sx, sy, 0, heuristic(sx, sy, gx, gy), null);
        Node goalNode = new Node(gx, gy, 0, heuristic(gx, gy, sx, sy), null);

        openF.add(startNode);
        openB.add(goalNode);

        bestGF.put(key(sx, sy), 0.0);
        bestGB.put(key(gx, gy), 0.0);

        parentF.put(key(sx, sy), startNode);
        parentB.put((key(gx, gy)), goalNode);

        while (!openF.isEmpty() && !openB.isEmpty()){
            double minF = openF.peek().f;
            double minB = openB.peek().f;
            if (minF + minB >= bestCost) break;

            if(minF <= minB){
                Node current = openF.poll();
                int cKey = key(current.x, current.y);

                if(current.g > bestGF.getOrDefault((cKey), Double.MAX_VALUE)) continue;

                if (bestGB.containsKey(cKey)){
                    double candidate = current.g + bestGB.get(cKey);
                    if (candidate < bestCost){
                        bestCost = candidate;
                        meetingKey = cKey;
                    }
                }

                for(int[] dir : NEIGHBORS){
                    int nx = current.x + dir [0];
                    int ny = current.y + dir [1];
                    if (!grid.isWalkable(nx, ny)) continue;

                    double stepCost = (dir[0] != 0 && dir[1] != 0) ? DIAGONAL_COST : STRAIGHT_COST;
                    double ng = current.g + stepCost;
                    int nKey = key(nx, ny);

                    if (ng >= bestGF.getOrDefault(nKey, Double.MAX_VALUE)) continue;

                    bestGF.put(nKey, ng);
                    double nf = ng + heuristic(nx, ny, gx, gy);
                    Node neighbor = new Node(nx, ny, ng, nf, current);
                    parentF.put(nKey, neighbor);
                    openF.add(neighbor);
                }
            } else {
                Node current = openB.poll();
                int cKey = key(current.x, current.y);

                if(current.g > bestGB.getOrDefault((cKey), Double.MAX_VALUE)) continue;

                if (bestGF.containsKey(cKey)){
                    double candidate = current.g + bestGF.get(cKey);
                    if (candidate < bestCost){
                        bestCost = candidate;
                        meetingKey = cKey;
                    }
                }

                for(int[] dir : NEIGHBORS){
                    int nx = current.x + dir [0];
                    int ny = current.y + dir [1];
                    if (!grid.isWalkable(nx, ny)) continue;

                    double stepCost = (dir[0] != 0 && dir[1] != 0) ? DIAGONAL_COST : STRAIGHT_COST;
                    double ng = current.g + stepCost;
                    int nKey = key(nx, ny);

                    if (ng >= bestGB.getOrDefault(nKey, Double.MAX_VALUE)) continue;

                    bestGB.put(nKey, ng);
                    double nf = ng + heuristic(nx, ny, sx, sy);
                    Node neighbor = new Node(nx, ny, ng, nf, current);
                    parentB.put(nKey, neighbor);
                    openB.add(neighbor);
                }
            }
        }
        if (meetingKey == -1) return Collections.emptyList();
        return reconstructBidirectional(meetingKey, parentF, parentB);
    }

    private List<double[]> reconstructBidirectional(int meetingKey,
                                                    HashMap<Integer, Node> parentF,
                                                    HashMap<Integer, Node> parentB){
        List<double[]> path = new ArrayList<>();

        Node node = parentF.get(meetingKey);
        while (node != null){
            path.add(new double[]{ grid.toInches(node.x), grid.toInches(node.y) });
            node = node.parent;
        }
        Collections.reverse(path);

        node = parentB.get(meetingKey);
        if (node != null) node = node.parent;
        while (node != null) {
            path.add(new double[]{ grid.toInches(node.x), grid.toInches(node.y) });
            node = node.parent;
        }
        return path;
    }

    private double heuristic(int x, int y, int gx, int gy) {
        int dx = Math.abs(x - gx);
        int dy = Math.abs(y - gy);
        return STRAIGHT_COST * (dx + dy) + (DIAGONAL_COST - 2 * STRAIGHT_COST) * Math.min(dx, dy);
    }

    private List<double[]> reconstruct(Node node) {
        List<double[]> path = new ArrayList<>();
        while (node != null) {
            path.add(new double[]{ grid.toInches(node.x), grid.toInches(node.y) });
            node = node.parent;
        }
        Collections.reverse(path);
        return path;
    }

    private int key(int x, int y) {
        return y * PathGrid.GRID_SIZE + x;
    }
}