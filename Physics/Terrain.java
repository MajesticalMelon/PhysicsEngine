package Physics;

import java.util.ArrayList;

import java.awt.Graphics2D;

public class Terrain {
    private ArrayList<Vector2D> terrainPoints;
    private ArrayList<Vector2D> terrainEdges;

    public Terrain(ArrayList<Vector2D> points) {
        terrainPoints = points;

        // Create edges
        terrainEdges = new ArrayList<>();
        
        for (int i = 1; i < points.size(); i++) {
            terrainEdges.add(Vector2D.sub(terrainPoints.get(i), terrainPoints.get(i - 1)));
        }
    }

    public Terrain() {
        this(new ArrayList<>());
    }

    public ArrayList<Vector2D> getTerrain() {
        return terrainPoints;
    }

    public void addPoint(Vector2D v) {
        if (terrainPoints.size() != 0) {
        terrainEdges.add(Vector2D.sub(v, terrainPoints.get(terrainPoints.size() - 1)));
        }
        
        terrainPoints.add(v);
    }

    public void draw(Graphics2D g2) {
        for (int i = 1; i < terrainPoints.size(); i++) {
            Vector2D point0 = terrainPoints.get(i - 1);
            Vector2D point1 = terrainPoints.get(i);

            g2.drawLine(
                (int)point0.getX(),
                (int)point0.getY(),
                (int)point1.getX(),
                (int)point1.getY()
            );
        }
    }

    public void drawNormals(Graphics2D g2) {
        for (int i = 0; i < terrainEdges.size(); i++) {
            Vector2D point0 = terrainPoints.get(i);
            Vector2D edge = terrainEdges.get(i);

            Vector2D midPoint = Vector2D.add(point0, Vector2D.div(edge, 2));

            Vector2D normal = edge.normal2();
            normal.mult(50);

            g2.drawLine(
                (int)midPoint.getX(),
                (int)midPoint.getY(),
                (int)midPoint.getX() + (int)normal.getX(),
                (int)midPoint.getY() + (int)normal.getY()
            );
        }
    }
}
