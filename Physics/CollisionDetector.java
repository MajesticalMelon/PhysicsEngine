package Physics;

import java.util.ArrayList;


public class CollisionDetector {
    
    ArrayList<RigidBody> bodies = new ArrayList<>();

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

                if (SAT(a, b)) {
                    a.collide(b);
                }
            } else {
                i += 2;
            }
        }
    }

    //Implementation of the Seperated Axis Theorem
    public boolean SAT(RigidBody a, RigidBody b) {
        Vector2D perpLine;
        double dot;
        ArrayList<Vector2D> perpStack = new ArrayList<>();
        double aMin = 9999999999999999d;
        double aMax = -9999999999999999d;
        double bMin = 9999999999999999d;
        double bMax = -9999999999999999d;

        //Calculate vectors perpendicular to each polygon's edges
        for (int i = 0; i < a.getEdges().size(); i++) {
            perpLine = new Vector2D(
                -a.edges.get(i).getY(), 
                a.edges.get(i).getX()
            );

            perpStack.add(perpLine);
        }

        for (int i = 0; i < b.getEdges().size(); i++) {
            perpLine = new Vector2D(
                -b.edges.get(i).getY(), 
                b.edges.get(i).getX()
            );

            perpStack.add(perpLine);
        }

        
        for (int i = 0; i < perpStack.size(); i++) {
            //Calculate dot products between polygon's points and their perpLines
            //Describes the location of the polyogn's points along an infinite perpendicular line
            for (int j = 0; j < a.getPoints().size(); j++) {
                dot = Vector2D.dot(a.getPoints().get(j), perpStack.get(i));

                if (dot > aMax) {
                    aMax = dot;
                }

                if (dot < aMin) {
                    aMin = dot;
                }
            }

            for (int j = 0; j < a.getPoints().size(); j++) {
                dot = Vector2D.dot(b.getPoints().get(j), perpStack.get(i));

                if (dot > bMax) {
                    bMax = dot;
                }

                if (dot < bMin) {
                    bMin = dot;
                }
            }

            if ((aMin < bMax && aMin > bMin) || (bMin < aMax && bMin > aMin)) {
                continue;
            } else {
                return false;
            }
        }
        return true;
    }
}