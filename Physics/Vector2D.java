package Physics;

public class Vector2D {
    private float x, y;

    public Vector2D(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void set(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public void set(Vector2D v) {
        this.x = v.x;
        this.y = v.y;
    }

    //Vector Addition
    public static Vector2D add(Vector2D v1, Vector2D v2) {
        return new Vector2D(v1.x + v2.x, v1.y + v2.y);
    }

    public static Vector2D add(Vector2D v1, float n) {
        return new Vector2D(v1.x + n, v1.y + n);
    }

    public void add(float n) {
        this.x += n;
        this.y += n;
    }

    public void add(float x, float y) {
        this.x += x;
        this.y += y;
    }

    public void add(Vector2D v) {
        this.x += v.x;
        this.y += v.y;
    }

    //Vector Subtraction
    public static Vector2D sub(Vector2D v1, Vector2D v2) {
        return new Vector2D(v1.x - v2.x, v1.y - v2.y);
    }

    public static Vector2D sub(Vector2D v1, float n) {
        return new Vector2D(v1.x - n, v1.y - n);
    }

    public void sub(float n) {
        this.x -= n;
        this.y -= n;
    }

    public void sub(float x, float y) {
        this.x -= x;
        this.y -= y;
    }

    public void sub(Vector2D v) {
        this.x -= v.x;
        this.y -= v.y;
    }

    //Vector Multiplication
    public static Vector2D mult(Vector2D v1, Vector2D v2) {
        return new Vector2D(v1.x * v2.x, v1.y * v2.y);
    }

    public static Vector2D mult(Vector2D v1, float n) {
        return new Vector2D(v1.x * n, v1.y * n);
    }

    public void mult(float n) {
        this.x *= n;
        this.y *= n;
    }

    public void mult(float x, float y) {
        this.x *= x;
        this.y *= y;
    }

    public void mult(Vector2D v) {
        this.x *= v.x;
        this.y *= v.y;
    }

    //Vector Division
    public static Vector2D div(Vector2D v1, Vector2D v2) {
        return new Vector2D(v1.x / v2.x, v1.y / v2.y);
    }

    public static Vector2D div(Vector2D v1, float n) {
        return new Vector2D(v1.x / n, v1.y / n);
    }

    public void div(float n) {
        this.x /= n;
        this.y /= n;
    }

    public void div(float x, float y) {
        this.x /= x;
        this.y /= y;
    }

    public void div(Vector2D v) {
        this.x /= v.x;
        this.y /= v.y;
    }

    public float mag() {
        return (float) Math.sqrt(this.x * this.x + this.y * this.y);
    }

    //Dot Product
    public float dot(Vector2D v) {
        return this.x * v.x + this.y * v.y;
    }

    public static float dot(Vector2D v1, Vector2D v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    public static Vector2D norm(Vector2D v) {
        if (v.mag() == 0f) {
            return new Vector2D(0, 0);
        } else {
            return Vector2D.div(v, v.mag());
        }
    }

    public static float angleBetween(Vector2D v1, Vector2D v2) {
        return (float) (Math.atan2(v2.y, v2.x) - Math.atan2(v1.y, v1.x));
    }

    public void rotate(float radians) {
        Vector2D rotated = new Vector2D(
            this.x * (float)Math.cos(radians) - this.y * (float)Math.sin(radians),
            this.x * (float)Math.sin(radians) + this.y * (float)Math.cos(radians)
            );

        this.x = rotated.x;
        this.y = rotated.y;
    }

    public static Vector2D rotate(Vector2D v, float radians) {
        return new Vector2D(
            v.x * (float)Math.cos(radians) - v.y * (float)Math.sin(radians),
            v.x * (float)Math.sin(radians) + v.y * (float)Math.cos(radians)
            );
    }

    public Vector2D normal1() {
        return norm(new Vector2D(-y, x));
    }

    public Vector2D normal2() {
        return norm(new Vector2D(y, -x));
    }

    public float getX() {
        return this.x;
    }

    public float getY() {
        return this.y;
    }

    public Vector2D get() {
        return this;
    }
}