package Physics;

import java.util.ArrayList;

public class CollisionDetector {
    ArrayList<RigidBody> bodies = new ArrayList<>();
    public Vector2D min = new Vector2D(0, 0);

    public CollisionDetector(ArrayList<RigidBody> s) {
        bodies = s;
    }

    private int partition(ArrayList<Vector2D> arr, int low, int high) {
        float pivot = arr.get(high).getX();
        int i = (low - 1); // index of smaller element
        for (int j = low; j < high; j++) {
            // If current element is smaller than the pivot
            if (arr.get(j).getX() < pivot) {
                i++;

                // swap arr[i] and arr[j]
                Vector2D temp = arr.get(i);
                arr.set(i, arr.get(j));
                arr.set(j, temp);
            }
        }

        // swap arr[i+1] and arr[high] (or pivot)
        Vector2D temp = arr.get(i + 1);
        arr.set(i + 1, arr.get(high));
        arr.set(high, temp);

        return i + 1;
    }

    // The main function that implements QuickSort()
    // arr[] --> Array to be sorted,
    // low --> Starting index,
    // high --> Ending index
    private void sort(ArrayList<Vector2D> arr, int low, int high) {
        if (low < high) {
            // pi is partitioning index, arr[pi] is
            // now at right place
            int pi = partition(arr, low, high);

            // Recursively sort elements before
            // partition and after partition
            sort(arr, low, pi - 1);
            sort(arr, pi + 1, high);
        }
    }

    public void detectCollision(ArrayList<RigidBody> s) {
        // Sort each rigidbody's min and max X positions
        ArrayList<Vector2D> posX = new ArrayList<>();
        for (int i = 0; i < s.size(); i++) {
            RigidBody body = s.get(i);

            // i is the y component in order to keep track of which point bleongs to which
            // body when sorted
            posX.add(new Vector2D(body.getMinX(), i));
            posX.add(new Vector2D(body.getMaxX(), i));
        }

        sort(posX, 0, posX.size() - 1);

        // Loop through the sorted X pos's and check
        // if 2 consecutive points are the same
        for (int i = 0; i < posX.size() - 1; i++) {
            if ((int) posX.get(i).getY() != (int) posX.get(i + 1).getY()) {
                RigidBody a = s.get((int) posX.get(i).getY());
                RigidBody b = s.get((int) posX.get(i + 1).getY());

                // Make sure at least one body is dynamic
                if (a.getType() == BodyType.Dynamic || b.getType() == BodyType.Dynamic) {
                    // Detect and resolve collisions between two bodies
                    a.setCollisionState(SAT(a, b));
                }
            }
        }
    }

    // Implementation of the Seperated Axis Theorem
    private boolean SAT(RigidBody a, RigidBody b) {
        float dot;
        ArrayList<Vector2D> perpStack = new ArrayList<>();

        float overlap = Float.MAX_VALUE;
        Vector2D smallestAxis = new Vector2D(0, 0);
        RigidBody axisBody = null;

        // Calculate vectors perpendicular to each polygon's edges

        // Get the normals for a
        for (int i = 0; i < a.getEdges().size(); i++) {
            perpStack.add(a.getEdges().get(i).normal1());
        }

        // Get the normals for b
        for (int i = 0; i < b.getEdges().size(); i++) {
            perpStack.add(b.getEdges().get(i).normal1());
        }

        for (int i = 0; i < perpStack.size(); i++) {
            // Guarantee that the first dot products are less than the min/greater than the
            // max
            float aMin = Float.POSITIVE_INFINITY;
            float aMax = Float.NEGATIVE_INFINITY;
            float bMin = Float.POSITIVE_INFINITY;
            float bMax = Float.NEGATIVE_INFINITY;

            // Calculate dot products between polygon's points and their perpLines
            // which describes the location of the polyogn's points along an infinite
            // perpendicular line
            //// Projects each polygon ////
            for (int j = 0; j < a.getPoints().size(); j++) {
                dot = Vector2D.dot(perpStack.get(i), a.getPoints().get(j));

                if (dot < aMin) {
                    aMin = dot;
                }

                if (dot > aMax) {
                    aMax = dot;
                }
            }

            for (int j = 0; j < b.getPoints().size(); j++) {
                dot = Vector2D.dot(perpStack.get(i), b.getPoints().get(j));

                if (dot < bMin) {
                    bMin = dot;
                }

                if (dot > bMax) {
                    bMax = dot;
                }
            }

            // Check if bounds of dot products for each polygon intersect
            // Continue to check until out of points or until the condition is not met
            if ((aMin < bMax && aMin > bMin) || (aMax > bMin && aMax < bMax)) {

                if (aMin < bMax && aMin > bMin) {
                    float currentOverlap = Math.abs(aMin - bMax);

                    if (currentOverlap < overlap) {
                        overlap = currentOverlap;
                        smallestAxis = perpStack.get(i); // Assign the direction of translation
                    }
                }

                if (aMax > bMin && aMax < bMax) {
                    // Get the minimum amount of overlap in order to calculate
                    // the min translation vec
                    float currentOverlap = Math.abs(aMax - bMin);

                    if (currentOverlap < overlap) {
                        overlap = currentOverlap;
                        smallestAxis = perpStack.get(i); // Assign the direction of translation
                    }
                }

                if (i < 4) {
                    axisBody = a;
                } else {
                    axisBody = b;
                }

                continue;
            } else {
                return false;
            }
        }

        // Calculate the Minimum Translation Vector
        RigidBody oppBody;
        Vector2D mtvA;
        Vector2D mtvB;

        if (axisBody == b) {
            oppBody = a;
            mtvA = Vector2D.mult(smallestAxis, overlap);
            mtvB = Vector2D.mult(smallestAxis, -overlap);
        } else {
            oppBody = b;
            mtvA = Vector2D.mult(smallestAxis, -overlap);
            mtvB = Vector2D.mult(smallestAxis, overlap);
        }
        
        // Find the collision point
        float maxDist = Float.MIN_VALUE;
        Vector2D collisionPoint = new Vector2D(0, 0);

        // Project each point onto the axisBody's position vector
        for (Vector2D point : oppBody.getPoints()) {
            // The greatest scalar is closest to the axisBody
            // and therefore the collision point
            float projection = Vector2D.scalarProject(point, axisBody.getPos());

            if (projection > maxDist) {
                maxDist = projection;
                collisionPoint = point;
            }
        }

        // Apply translation
        if (a.getType() == BodyType.Dynamic && b.getType() == BodyType.Dynamic) {
            a.addPos(mtvA);
            b.addPos(mtvB);
        } else if (a.getType() == BodyType.Dynamic) {
            a.addPos(mtvA);
        } else {
            b.addPos(mtvB);
        }

        

        // Apply forces
        float forceScalar = (a.getLinearMomentum().mag() + b.getLinearMomentum().mag()) / (a.getMass() + b.getMass());

        //a.setLinearVelocity(new Vector2D(0, 0));
        //b.setLinearVelocity(new Vector2D(0, 0));

        a.applyForce(Vector2D.mult(mtvA, forceScalar), Vector2D.sub(collisionPoint, a.getPos()));
        b.applyForce(Vector2D.mult(mtvB, forceScalar), Vector2D.sub(collisionPoint, b.getPos()));

        return true;
    }
}