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
        return Vector2D.div(v, v.mag());
    }

    public static float angleBetween(Vector2D v1, Vector2D v2) {
        return (float) (Math.atan2(v2.y, v2.x) - Math.atan2(v1.y, v1.x));
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