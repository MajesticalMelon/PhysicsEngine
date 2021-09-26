package Physics;

import java.util.ArrayList;

public class CollisionDetector {
    ArrayList<RigidBody> bodies = new ArrayList<>();
    ArrayList<Terrain> terrains = new ArrayList<>();

    public CollisionDetector(ArrayList<RigidBody> s, ArrayList<Terrain> t) {
        bodies = s;
        terrains = t;
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
        //  if 2 consecutive points are the same
        for (int i = 0; i < posX.size() - 1; i++) {
            if ((int) posX.get(i).getY() != (int) posX.get(i + 1).getY()) {

                RigidBody a = s.get((int) posX.get(i).getY());
                RigidBody b = s.get((int) posX.get(i + 1).getY());

                // Detect and resolve collisions between two bodies
                SAT(a, b);
            }
        }

        // Check every rigidbody against every terrain
        for (Terrain terra : terrains) {
            for (RigidBody a : s) {
                // Detect and resolve collisions between body and terrain
                a.setCollisionState(terrainCollision(a, terra));
            }
        }
    }

    // Implementation of the Seperated Axis Theorem
    private boolean SAT(RigidBody a, RigidBody b) {
        float dot;
        ArrayList<Vector2D> perpStack = new ArrayList<>();

        float overlap = Float.MAX_VALUE;
        Vector2D smallestAxis = new Vector2D(0, 0);

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

                continue;
            } else {
                return false;
            }
        }

        Vector2D aToB = Vector2D.sub(b.getPos(), a.getPos());
        Vector2D bToA = Vector2D.sub(a.getPos(), b.getPos());

        Vector2D collisionPoint = new Vector2D(0, 0); // default
        float smallestAngle = Float.POSITIVE_INFINITY;
        boolean aPoint = false; // Determines whether collision point came from A or B

        // Find a collision point
        for (Vector2D point : a.getPoints()) {
            // Get the angle between the vector pointing from
            // body a to body b and the vector that describes point
            float testAngle = Math.abs(Vector2D.angleBetween(aToB, point)) % (float)Math.PI;

            // Compare the calulated angle to the current smallest angle
            if (testAngle < smallestAngle) {
                smallestAngle = testAngle;
                collisionPoint = point;
                aPoint = true;
            }
        }

        // Find a collision point
        for (Vector2D point : b.getPoints()) {
            float testAngle = Math.abs(Vector2D.angleBetween(bToA, point)) % (float)Math.PI;

            if (testAngle < smallestAngle) {
                smallestAngle = testAngle;
                collisionPoint = point;
                aPoint = false;
            }
        }

        // Calculate the Minimum Translation Vector
        Vector2D mtv = Vector2D.mult(smallestAxis, overlap);
        mtv.set(Math.abs(mtv.getX()), Math.abs(mtv.getY()));

        // Get direction of translation for both objects
        Vector2D aDir = new Vector2D(
            Math.signum(bToA.getX()),
            Math.signum(bToA.getY())
            );

        Vector2D bDir = new Vector2D(
            Math.signum(aToB.getX()),
            Math.signum(aToB.getY())
            );
        
        // Apply translation
        a.addPos(Vector2D.mult(mtv, aDir));
        b.addPos(Vector2D.mult(mtv, bDir));

        // Get edge normal
        ArrayList<Vector2D> edges = aPoint ? b.getEdges() : a.getEdges();
        Vector2D collisionPointer = aPoint ? Vector2D.sub(collisionPoint, b.getPos()) : Vector2D.sub(collisionPoint, a.getPos());

        smallestAngle = Float.POSITIVE_INFINITY;
        Vector2D normalEdge = new Vector2D(0f, 0f);

        // Check edge normals for which one points toward the collision point most
        for (Vector2D edge : edges) {
            float angle = Math.abs(Vector2D.angleBetween(collisionPointer, edge.normal1())) % (float)Math.PI;

            if (angle < smallestAngle) {
                smallestAngle = angle;
                normalEdge = edge.normal1();
            }
        }

        float forceScalar = a.getLinearVelocity().mag() * (mtv.mag() * a.getMass());

        a.setLinearVelocity(new Vector2D(0f, 0f));
        b.setLinearVelocity(new Vector2D(0f, 0f));

        normalEdge.mult(forceScalar);

        a.applyForce(normalEdge, Vector2D.sub(collisionPoint, a.getPos()));
        b.applyForce(Vector2D.mult(normalEdge, -1f), Vector2D.sub(collisionPoint, b.getPos()));

        return true;
    }

    // TODO: Fix terrain collision detection with vertical sections
    // TODO: Possibly switch terrain to a height map system
    // Collide with the terrain
    private boolean terrainCollision(RigidBody a, Terrain terra) {
        // Find the the terrain points to the left and right of the body
        // Indices of the closest terrain points outside of the body
        int terrainLeft = 0;
        int terrainRight = 0;

        float minDifference1 = Float.POSITIVE_INFINITY;
        float minDifference2 = Float.POSITIVE_INFINITY;

        for (int i = 0; i < terra.getTerrain().size(); i++) {
            // Save the x dimnension
            float x = terra.getTerrain().get(i).getX();

            // Check if the position is on the left or right side
            if (a.getMinX() - x > 0 && a.getMinX() - x < minDifference1) {
                minDifference1 = a.getMinX() - x;
                terrainLeft = i;
            } else if (x - a.getMaxX() > 0 && x - a.getMaxX() < minDifference2) {
                minDifference2 = x - a.getMaxX();
                terrainRight = i;
            } else {
                continue;
            }
        }

        boolean collision = false;

        // Check if any point of the body is below the
        // previously calculated terrain points
        for (int i = terrainLeft; i < terrainRight; i++) {
            Vector2D terrainPoint = terra.getTerrain().get(i);
            Vector2D terrainEdge = terra.getEdges().get(i);

            float minDist = Float.POSITIVE_INFINITY;

            // Loop through all points in the body
            // and check for collision
            for (Vector2D point : a.getPoints()) {
                Vector2D tPointToRBPoint = Vector2D.sub(point, terrainPoint);

                float angleToNormal = Vector2D.angleBetween(tPointToRBPoint, terra.getNormals().get(i));

                if (Math.abs(angleToNormal) > Math.PI / 2 && Math.abs(angleToNormal) < 3f * Math.PI / 2f) {
                    // Calculate minimum distance
                    float angleToEdge = Vector2D.angleBetween(tPointToRBPoint, terrainEdge);
                    minDist = Math.abs(Math.min(minDist, tPointToRBPoint.mag() * (float) Math.sin(angleToEdge)));

                    // Apply rebounding force //
                    float forceScalar = 2 * RigidBody.GRAVITY.mag() * a.getLinearMomentum().mag() * (minDist * a.getMass());
                    //                     Counteract gravity                        scale force depending on how far it gets
                    a.applyForce(Vector2D.mult(terra.getNormals().get(i), forceScalar),
                            Vector2D.sub(point, a.getPos()));
                }
            }

            // Adjust position if there was a collision
            if (minDist < Float.POSITIVE_INFINITY) {
                a.addPos(Vector2D.mult(terra.getNormals().get(i), minDist));
                collision = true;
            }
        }

        return collision;
    }
}