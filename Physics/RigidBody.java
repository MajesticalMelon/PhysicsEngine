package Physics;

import java.awt.Polygon;
import java.util.ArrayList;

public class RigidBody {
    //Initial variables
    double width, height, mass;
    ArrayList<Vector2D> points = new ArrayList<>();
    ArrayList<Vector2D> edges = new ArrayList<>();

    //Linear variables
    Vector2D pos;
    Vector2D linVel = new Vector2D(0, 0);
    Vector2D linAcc = new Vector2D(0, 0);

    //Angular variables
    double rotation = 0;
    double angVel = 0;
    double angAcc = 0;

    //Physics variables
    double momentOfInertia;
    double distanceFromPivot;
    double momentumScalar = 0.5;
    Vector2D force = new Vector2D(0, 0);

    //Collision variables
    double maxX = 0;
    double maxY = 0;
    double minX = 0;
    double minY = 0;

    public RigidBody(double cx, double cy, double w, double h, double m) {
        this.pos = new Vector2D(cx, cy);
        this.width = w;
        this.height = h;
        this.mass = m;
        this.momentOfInertia = m * w * h;

        maxX = cx + w/2;
        maxY = cy + h/2;

        minX = cx - w/2;
        minY = cy - h/2;

        points.add(new Vector2D(minX, minY));
        points.add(new Vector2D(maxX, minY));
        points.add(new Vector2D(maxX, maxY));
        points.add(new Vector2D(minX, maxY));

        edges.clear();
        calculateEdges();
    }

    public void update() {
        //Update position
        this.pos.add(this.linVel);
        this.linVel.add(this.linAcc);

        //Update angle
        this.rotation += this.angVel;
        this.angVel += this.angAcc;

        if (this.rotation > 2 * Math.PI || this.rotation < -2 * Math.PI) {
            this.rotation = 0;
        }

        movePoints();

        force = Vector2D.mult(this.linAcc, this.mass);

        arrangePoints();

        edges.clear();  
        calculateEdges();

        this.linAcc = new Vector2D(0, 0);
        this.angAcc = 0;
    }

    public void applyForce(Vector2D force, Vector2D forcePos) {
        double angleBetween = Math.atan2(force.getY(), force.getX()) - Math.atan2(forcePos.getY(), forcePos.getX());

        this.angAcc += (force.mag() * forcePos.mag() * Math.sin(angleBetween)) / this.getMoment();

        force.div(this.mass);
        this.linAcc.add(force);
    }

    private void conserveLinearMomentum(RigidBody j) {
        if (this.getMass() > j.getMass()) {
            Vector2D v1f = Vector2D.add(Vector2D.mult(this.getLinearVelocity(), this.momentumScalar), this.getLinearVelocity());
            Vector2D v2 = j.getLinearVelocity();
            Vector2D v2f = Vector2D.sub(v1f, v2);
            j.setLinearVelocity(v2f);
            this.setLinearVelocity(v1f);
        } else {
            Vector2D v2f = Vector2D.add(Vector2D.mult(j.getLinearVelocity(), j.momentumScalar), j.getLinearVelocity());
            Vector2D v1 = this.getLinearVelocity();
            Vector2D v1f = Vector2D.sub(v2f, v1);
            j.setLinearVelocity(v1f);
            this.setLinearVelocity(v2f);
        }

        /*
        if (this.getLinearVelocity().getX() == 0) {
            this.setLinearVelocity(new Vector2D(0.001, 0.001));
        }
        if (j.getLinearVelocity().getX() == 0) {
            j.setLinearVelocity(new Vector2D(0.001, 0.001));
        }
        Vector2D m1 = Vector2D.mult(this.getLinearVelocity(), this.getMass());
        Vector2D m2 = Vector2D.mult(j.getLinearVelocity(), j.getMass());

        Vector2D p = Vector2D.add(m1, m2);

        if (this.getMass() > j.getMass()) {
            Vector2D m1f = Vector2D.mult(m1, (1 - this.momentumScalar));
            Vector2D pf = Vector2D.add(m1f, m2);
            Vector2D vf = Vector2D.div(pf, j.getMass());
            j.setLinearVelocity(vf);
            
            Vector2D m2f = Vector2D.mult(vf, j.getMass());
            p.sub(m2f);
            p.div(this.getMass());
            this.setLinearVelocity(p);
        } else {
            Vector2D m2f = Vector2D.mult(m2, (1 - this.momentumScalar));
            Vector2D pf = Vector2D.add(m2f, m1);
            Vector2D vf = Vector2D.div(pf, this.getMass());
            this.setLinearVelocity(vf);

            Vector2D m1f = Vector2D.mult(this.getLinearVelocity(), this.getMass());
            p.sub(m1f);
            p.div(j.getMass());
            j.setLinearVelocity(p);
        }
        */
    }

    private void conserveAngularMomentum(RigidBody j) {
        double d1 = Math.sqrt((this.getPos().getX() * this.getPos().getX()) + (this.getPos().getY() * this.getPos().getY())) * Vector2D.angleBetween(this.getLinearVelocity(), this.getPos());
        double d2 = Math.sqrt((j.getPos().getX() * j.getPos().getX()) + (j.getPos().getY() * j.getPos().getY())) * Vector2D.angleBetween(j.getLinearVelocity(), j.getPos());
        
        double m1 = this.getMoment() * Vector2D.div(this.getLinearVelocity(), d1).mag();
        double m2 = j.getMoment() * Vector2D.div(j.getLinearVelocity(), d2).mag();

        double l = m1 + m2;

        if (this.getMoment() > j.getMoment()) {
            double m1f = m1 * (1 - this.momentumScalar);
            double lf = m1f + m2;
            double wf = lf / j.getMoment();
            j.setAngularVelocity(wf);

            double m2f = wf * j.getMoment();
            l -= m2f;
            l /= this.getMoment();
            j.setAngularVelocity(l);
        } else {
            double m2f = m2 * (1 - this.momentumScalar);
            double lf = m2f + m1;
            double wf = lf / this.getMoment();
            this.setAngularVelocity(wf);

            double m1f = wf * this.getMoment();
            l -= m1f;
            l /= j.getMoment();
            this.setAngularVelocity(l);
        }
    }

    public void collide(RigidBody j) {
        this.conserveLinearMomentum(j);
        //this.conserveAngularMomentum(j);
    }

    private void movePoints() {
        for (Vector2D v : points) {
            v.sub(this.getPos());
            double rotatedX = v.getX() * Math.cos(this.getAngularVelocity()) - v.getY() * Math.sin(this.getAngularVelocity());
            double rotatedY = v.getX() * Math.sin(this.getAngularVelocity()) + v.getY() * Math.cos(this.getAngularVelocity());
            v.set(rotatedX, rotatedY);
            v.add(this.getPos());
            v.add(this.getLinearVelocity());
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
            edges.add(Vector2D.sub(points.get(i), points.get(i + 1)));
        }
        edges.add(Vector2D.sub(points.get(points.size() - 1), points.get(0)));
    }

    public Vector2D getPos() {
        return this.pos;
    }

    public double getWidth() {
        return this.width;
    }

    public double getHeight() {
        return this.height;
    }

    public double getAngle() {
        return this.rotation;
    }

    public double getMass() {
        return this.mass;
    }

    public Vector2D getLinearVelocity() {
        return this.linVel;
    }

    public double getAngularVelocity() {
        return this.angVel;
    }

    public double getAngularAcceleration() {
        return this.angAcc;
    }

    public double getMoment() {
        return this.momentOfInertia;
    }

    public void setLinearVelocity(Vector2D v) {
        this.linVel = v;
    }

    public void setAngularVelocity(double w) {
        this.angVel = w;
    }

    public void setLinearAcceleration(Vector2D v) {
        this.linVel = v;
    }

    public void setAngularAcceleration(double w) {
        this.angVel = w;
    }

    public void setPos(Vector2D p) {
        this.pos = p;
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
     * @return the maxX
     */
    public double getMaxX() {
        return maxX;
    }

    /**
     * @return the maxY
     */
    public double getMaxY() {
        return maxY;
    }

    /**
     * @return the minX
     */
    public double getMinX() {
        return minX;
    }

    /**
     * @return the minY
     */
    public double getMinY() {
        return minY;
    }

    public Vector2D getForce() {
        return this.force;
    }

    public ArrayList<Vector2D> getPoints() {
        return this.points;
    }

    public ArrayList<Vector2D> getEdges() {
        return this.edges;
    }

    public Vector2D getMaxPoint() {
        return new Vector2D(getMaxX(), getMaxY());
    }
}