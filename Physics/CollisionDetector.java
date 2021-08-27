package Physics;

import java.util.ArrayList;


public class CollisionDetector {
    ArrayList<RigidBody> bodies = new ArrayList<>();
    Vector2D pocA = new Vector2D(0, 0);
    Vector2D pocB = new Vector2D(0, 0);

    Vector2D contactPoint;

    public CollisionDetector(ArrayList<RigidBody> s) {
        bodies = s;
    }
    
    int partition(ArrayList<Vector2D> arr, int low, int high) 
    { 
        double pivot = arr.get(high).getX();  
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

                boolean collision = SAT(a, b);
                System.out.println(collision);

                if (collision) {
                    //moveOut(a, b);
                    a.collide(b, pocA, pocB);
                }
            } else {
                i+=2;
            }
        }
    }

    //Implementation of the Seperated Axis Theorem
    public boolean SAT(RigidBody a, RigidBody b) {
        Vector2D perpLine;
        double dot;
        ArrayList<Vector2D> perpStack = new ArrayList<>();

        //Guarantee that the first dot products are less than the min or greater than the max
        Vector2D aMin = new Vector2D(Double.MAX_VALUE, 0);
        Vector2D aMax = new Vector2D(Double.MIN_VALUE, 0);
        Vector2D bMin = new Vector2D(Double.MAX_VALUE, 0);
        Vector2D bMax = new Vector2D(Double.MIN_VALUE, 0);

        //Calculate vectors perpendicular to each polygon's edges
        for (int i = 0; i < a.getEdges().size(); i++) {
            perpLine = new Vector2D(
                -a.getEdges().get(i).getY(), 
                a.getEdges().get(i).getX()
            );

            perpStack.add(Vector2D.norm(perpLine));
        }

        for (int i = 0; i < b.getEdges().size(); i++) {
            perpLine = new Vector2D(
                -b.getEdges().get(i).getY(), 
                b.getEdges().get(i).getX()
            );

            perpStack.add(Vector2D.norm(perpLine));
        }

        for (int i = 0; i < perpStack.size(); i++) {
            //Calculate dot products between polygon's points and their perpLines
            //Describes the location of the polyogn's points along an infinite perpendicular line
            for (int j = 0; j < a.getPoints().size(); j++) {
                dot = Vector2D.dot(perpStack.get(i), a.getPoints().get(j));

                if (dot < aMin.getX()) {
                    aMin.set(dot, j);
                } else if (dot > aMax.getX()) {
                    aMax.set(dot, j);
                }
            }

            for (int j = 0; j < a.getPoints().size(); j++) {
                dot = Vector2D.dot(perpStack.get(i), b.getPoints().get(j));

                if (dot < bMin.getX()) {
                    bMin.set(dot, j);
                } else if (dot > bMax.getX()) {
                    bMax.set(dot, j);
                }
            }

            //Check if bounds of dot products for each polygon intersect
            //Continue to check until out of points or until the condition is not met
            if ((aMin.getX() < bMax.getX() && aMin.getX() > bMin.getX()) || (aMax.getX() > bMin.getX() && aMax.getX() < bMax.getX())) {
                continue;
            } else {
                return false;
            }
        }

        Vector2D aMaxPoint;
        Vector2D aMinPoint;
        Vector2D bMaxPoint;
        Vector2D bMinPoint;

        aMaxPoint = a.getPoints().get((int) aMax.getY());
        aMinPoint = a.getPoints().get((int) aMin.getY());
        bMaxPoint = b.getPoints().get((int) bMax.getY());
        bMinPoint = b.getPoints().get((int) bMin.getY());

        double distAMaxToB = Vector2D.sub(aMaxPoint, b.getPos()).mag();
        double distBMaxToA = Vector2D.sub(bMaxPoint, a.getPos()).mag();
        double distAMinToB = Vector2D.sub(aMinPoint, b.getPos()).mag();
        double distBMinToA = Vector2D.sub(bMinPoint, a.getPos()).mag();

        double radiusToA = Math.min(distAMaxToB, distAMinToB);
        pocB = new Vector2D(radiusToA, radiusToA);

        double radiusToB = Math.min(distBMaxToA, distBMinToA);
        pocA = new Vector2D(radiusToB, radiusToB);

        //Return true if the for loop completes
        return true;
    }
}