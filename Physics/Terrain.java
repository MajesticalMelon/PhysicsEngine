package Physics;

import java.util.ArrayList;

public class Terrain {
    private ArrayList<Vector2D> terrainPoints;

    public Terrain(ArrayList<Vector2D> points) {
        terrainPoints = points;
    }

    public Terrain() {
        this(new ArrayList<>());
    }

    public ArrayList<Vector2D> getTerrain() {
        return terrainPoints;
    }

    public void addPoint(Vector2D v) {
        terrainPoints.add(v);
    }
}
