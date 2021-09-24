package Physics;

import java.awt.Polygon;
import java.awt.geom.AffineTransform;

import java.util.ArrayList;

public class RigidBody {
    // Gravity
    public static Vector2D GRAVITY = new Vector2D(0, 0.009f);;

    // Initial variables
    private float width, height, mass;
    private ArrayList<Vector2D> points = new ArrayList<>();
    private ArrayList<Vector2D> pointVels = new ArrayList<>();
    private ArrayList<Vector2D> edges = new ArrayList<>();
    private BodyType type;
    private AffineTransform transform;

    // Linear variables
    private Vector2D pos;
    private Vector2D linVel = new Vector2D(0, 0);
    private Vector2D linAcc = new Vector2D(0, 0);

    // Angular variables
    private float rotation = 0;
    private float angVel = 0;
    private float angAcc = 0;

    // Physics variables
    private float momentOfInertia;
    private Vector2D force;

    // Collision variables
    private float maxX = 0;
    private float maxY = 0;
    private float minX = 0;
    private float minY = 0;

    public RigidBody(float cx, float cy, float w, float h, float m) {
        this.pos = new Vector2D(cx, cy);
        this.width = w;
        this.height = h;
        this.mass = m;
        this.momentOfInertia = m * w * h;

        this.transform = AffineTransform.getTranslateInstance(cx, cy);
        this.transform.rotate(this.rotation, cx, cy);

        this.type = BodyType.Dynamic;

        maxX = cx + (w / 2);
        maxY = cy + (h / 2);

        minX = cx - (w / 2);
        minY = cy - (h / 2);

        points.add(new Vector2D(minX, minY));
        points.add(new Vector2D(maxX, minY));
        points.add(new Vector2D(maxX, maxY));
        points.add(new Vector2D(minX, maxY));

        for (int i = 0; i < points.size(); i++) {
            edges.add(Vector2D.sub(points.get((i + 1) % points.size()), points.get(i)));
        }
    }

    public void update() {
        if (this.type != BodyType.Static) {
            // Apply gravity
            this.linAcc.add(Vector2D.mult(RigidBody.GRAVITY, this.mass));

            // Update position
            this.pos.add(this.linVel);
            this.linVel.add(this.linAcc);

            // Update angle
            this.rotation += this.angVel;
            this.rotation %= 2f * Math.PI;
            this.angVel += this.angAcc;

            // Rotate points and apply velocities
            movePoints();
            calculateEdges(); // Recalcualte the edges

            // Update transform
            this.transform = AffineTransform.getTranslateInstance(this.pos.getX(), this.pos.getY());
            this.transform.rotate(this.rotation, this.pos.getX(), this.pos.getY());

            // Calculate the force on this object
            force = Vector2D.mult(this.linAcc, this.mass);

            // Determine the bounds of this object
            arrangePoints();

            // Reset accelerations
            this.linAcc = new Vector2D(0, 0);
            this.angAcc = 0;
        }
    }

    public void applyForce(Vector2D force, Vector2D forcePos) {
        float angleBetween = (float) (Math.atan2(force.getY(), force.getX())
                - Math.atan2(forcePos.getY(), forcePos.getX()));

        this.angAcc += (force.mag() * forcePos.mag() * Math.sin(angleBetween)) / this.getMoment();

        force.div(this.mass);
        this.linAcc.add(force);
    }

    public void collide(RigidBody j) {

    }

    private void movePoints() {
        this.pointVels.clear();
        for (Vector2D v : points) {
            Vector2D pointCopy = new Vector2D(v.getX(), v.getY());

            v.sub(this.pos);
            float rotatedX = (float) (v.getX() * Math.cos(this.getAngularVelocity())
                    - v.getY() * Math.sin(this.getAngularVelocity()));
            float rotatedY = (float) (v.getX() * Math.sin(this.getAngularVelocity())
                    + v.getY() * Math.cos(this.getAngularVelocity()));
            v.set(rotatedX, rotatedY);
            v.add(this.pos);
            v.add(this.getLinearVelocity());

            this.pointVels.add(Vector2D.sub(v, pointCopy));
        }
    }

    private void arrangePoints() {
        for (int i = 0; i < points.size() - 1; i++) {
            maxX = Math.max(points.get(i).getX(), points.get(i + 1).getX());
            maxY = Math.max(points.get(i).getY(), points.get(i + 1).getY());

            minX = Math.min(points.get(i).getX(), points.get(i + 1).getX());
            minY = Math.min(points.get(i).getY(), points.get(i + 1).getY());
        }
    }

    private void calculateEdges() {
        for (int i = 0; i < points.size() - 1; i++) {
            edges.get(i).set(Vector2D.sub(points.get((i + 1) % points.size()), points.get(i)));
        }
    }

    public Vector2D getPos() {
        return this.pos;
    }

    public ArrayList<Vector2D> getEdges() {
        return this.edges;
    }

    public float getWidth() {
        return this.width;
    }

    public float getHeight() {
        return this.height;
    }

    public float getAngle() {
        return this.rotation;
    }

    public float getMass() {
        return this.mass;
    }

    public Vector2D getLinearVelocity() {
        return this.linVel;
    }

    public float getAngularVelocity() {
        return this.angVel;
    }

    public float getAngularAcceleration() {
        return this.angAcc;
    }

    public float getMoment() {
        return this.momentOfInertia;
    }

    public BodyType getType() {
        return this.type;
    }

    public void setLinearVelocity(Vector2D v) {
        this.linVel = v;
    }

    public void setAngularVelocity(float w) {
        this.angVel = w;
    }

    public void setLinearAcceleration(Vector2D v) {
        this.linVel = v;
    }

    public void setAngularAcceleration(float w) {
        this.angVel = w;
    }

    public void setType(BodyType bodyType) {
        this.type = bodyType;
        this.mass = 1f;
    }

    public void addPos(Vector2D p) {
        if (this.type != BodyType.Static) {
            for (Vector2D v : this.points) {
                v.add(p);
            }
            this.pos.add(p);
        }
    }

    /**
     * @return The Linear Momentum of this body
    **/
    public Vector2D getLinearMomentum() {
        return Vector2D.mult(this.getLinearVelocity(), this.getMass());
    }

    /**
     * @return the rigidibody as polygon
     **/
    public Polygon getPolygon() {
        Polygon body = new Polygon();
        for (Vector2D v : this.points) {
            body.addPoint((int) v.getX(), (int) v.getY());
        }
        return body;
    }

    /**
     * @return The Affine Transformation containing this body's rotation
     */
    public AffineTransform getAffineTransform() {
        return transform;
    }

    /**
     * @return the maxX
     */
    public float getMaxX() {
        return maxX;
    }

    /**
     * @return the maxY
     */
    public float getMaxY() {
        return maxY;
    }

    /**
     * @return the minX
     */
    public float getMinX() {
        return minX;
    }

    /**
     * @return the minY
     */
    public float getMinY() {
        return minY;
    }

    public Vector2D getForce() {
        return this.force;
    }
    
    public ArrayList<Vector2D> getPoints() {
        return this.points;
    }

    public ArrayList<Vector2D> getPointVelocities() {
        return this.pointVels;
    }

    public Vector2D getMaxPoint() {
        return new Vector2D(getMaxX(), getMaxY());
    }

    public Vector2D getMinPoint() {
        return new Vector2D(getMinX(), getMinY());
    }
}