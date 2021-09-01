package Physics;

import java.util.ArrayList;


public class CollisionDetector {
    ArrayList<RigidBody> bodies = new ArrayList<>();
    public Vector2D collisionPoint = new Vector2D(0, 0); // default

    public CollisionDetector(ArrayList<RigidBody> s) {
        bodies = s;
    }
    
    int partition(ArrayList<Vector2D> arr, int low, int high) 
    { 
        float pivot = arr.get(high).getX();  
        int i = (low-1); // index of smaller element 
        for (int j=low; j<high; j++) 
        { 
            // If current element is smaller than the pivot 
            if (arr.get(j).getX() < pivot) 
            { 
                i++; 
  
                // swap arr[i] and arr[j] 
                Vector2D temp = arr.get(i); 
                arr.set(i, arr.get(j)); 
                arr.set(j, temp); 
            } 
        } 
  
        // swap arr[i+1] and arr[high] (or pivot) 
        Vector2D temp = arr.get(i+1); 
        arr.set(i+1, arr.get(high)); 
        arr.set(high, temp);
  
        return i+1; 
    } 
  
    // The main function that implements QuickSort() 
    //   arr[] --> Array to be sorted, 
    //   low  --> Starting index, 
    //   high  --> Ending index 
    void sort(ArrayList<Vector2D> arr, int low, int high) 
    { 
        if (low < high) 
        { 
            // pi is partitioning index, arr[pi] is  
             // now at right place
            int pi = partition(arr, low, high); 
  
            // Recursively sort elements before 
            // partition and after partition 
            sort(arr, low, pi-1); 
            sort(arr, pi+1, high); 
        } 
    } 

    public void detectCollision(ArrayList<RigidBody> s) {
        ArrayList<Vector2D> posX = new ArrayList<>();
        for (int i = 0; i < s.size(); i++) {
            RigidBody body = s.get(i);

            //i is the y component in order to keep track of which point bleongs to which body when sorted
            posX.add(new Vector2D(body.getMinX(), i));
            posX.add(new Vector2D(body.getMaxX(), i));
        }

        sort(posX, 0, posX.size() - 1);

        for (int i = 0; i < posX.size() - 1; i++) {
            if ((int) posX.get(i).getY() != (int) posX.get(i + 1).getY()) {

                RigidBody a = s.get((int) posX.get(i).getY());
                RigidBody b = s.get((int) posX.get(i + 1).getY());

                // Detect and resolve collisions
                SAT(a, b);
            }
        }
    }

    // Implementation of the Seperated Axis Theorem
    public void SAT(RigidBody a, RigidBody b) {
        Vector2D perpLine;
        float dot;
        ArrayList<Vector2D> perpStack = new ArrayList<>();

        float overlap = Float.MAX_VALUE;
        Vector2D smallestAxis = new Vector2D(0, 0);

        //Calculate vectors perpendicular to each polygon's edges

        // Get the normals for a
        for (int i = 0; i < a.getEdges().size(); i++) {
            perpLine = new Vector2D(
                -a.getEdges().get(i).getY(), 
                a.getEdges().get(i).getX()
            );

            perpStack.add(Vector2D.norm(perpLine));
        }

        // Get the normals for b
        for (int i = 0; i < b.getEdges().size(); i++) {
            perpLine = new Vector2D(
                -b.getEdges().get(i).getY(), 
                b.getEdges().get(i).getX()
            );

            perpStack.add(Vector2D.norm(perpLine));
        }

        for (int i = 0; i < perpStack.size(); i++) {
            //Guarantee that the first dot products are less than the min/greater than the max
            float aMin = Float.POSITIVE_INFINITY;
            float aMax = Float.NEGATIVE_INFINITY;
            float bMin = Float.POSITIVE_INFINITY;
            float bMax = Float.NEGATIVE_INFINITY;

            // Calculate dot products between polygon's points and their perpLines
            // Describes the location of the polyogn's points along an infinite perpendicular line
            //// Projects each polygon
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

            //Check if bounds of dot products for each polygon intersect
            //Continue to check until out of points or until the condition is not met
            if ((aMin < bMax && aMin > bMin) 
            || (aMax > bMin && aMax < bMax)
            || (bMin < aMax && bMin > aMin)
            || (bMax < aMax && bMax > aMin)) {
                // Get the minimum amount of overlap in order to calculate
                // the min translation vec
                float currentOverlap = Math.min(aMax - bMin, bMax - aMin);
                
                if (currentOverlap < overlap) {
                    overlap = currentOverlap;
                    smallestAxis = perpStack.get(i); // Assign the direction of translation
                }

                continue;
            } else {
                return;
            }
        }

        Vector2D aToB = Vector2D.sub(b.getPos(), a.getPos());
        Vector2D bToA = Vector2D.sub(a.getPos(), b.getPos());

        float smallestAngle = Float.POSITIVE_INFINITY;

        // Find a collision point
        for (Vector2D point : a.getPoints()) {
            float testAngle = Math.abs(Vector2D.angleBetween(aToB, point));

            if (testAngle > Math.PI) {
                testAngle -= Math.PI;
            }

            if (testAngle < smallestAngle) {
                smallestAngle = testAngle;
                collisionPoint = point;
            }
        }

        // Find a collision point
        for (Vector2D point : b.getPoints()) {
            float testAngle = Math.abs(Vector2D.angleBetween(bToA, point));

            if (testAngle > Math.PI) {
                testAngle -= Math.PI;
            }

            if (testAngle < smallestAngle) {
                smallestAngle = testAngle;
                collisionPoint = point;
            }
        }

        // Calculate the Minimum Translation Vector
        Vector2D mtv = Vector2D.mult(smallestAxis, overlap);
        mtv.div(2f);

        a.applyForce(mtv, Vector2D.sub(collisionPoint, a.getPos()));

        mtv.mult(-1f);
        b.applyForce(mtv, Vector2D.sub(collisionPoint, b.getPos()));
    }
}