package Display;

import java.awt.event.*;
import java.util.ArrayList;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JPanel;
import javax.swing.Timer;

import Physics.RigidBody;
import Physics.Vector2D;
import Physics.CollisionDetector;

public class Canvas extends JPanel implements ActionListener {
    private static final long serialVersionUID = 1L;

    public ArrayList<RigidBody> shapes = new ArrayList<>();

    public Timer gameTimer = new Timer(1000/144, this);
    private int delay = 0;

    CollisionDetector CD;

    RigidBody square = new RigidBody(100, 350, 50, 50, 10);
    RigidBody rectangle = new RigidBody(320, 350, 100, 200, 5);

    public Canvas() {
        setBackground(new Color(50, 50, 50));

        shapes.add(square);
        shapes.add(rectangle);

        shapes.get(0).applyForce(new Vector2D(15, 0), new Vector2D(5, 2));
        
        CD = new CollisionDetector(this.shapes);
        gameTimer.start();
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2 = (Graphics2D) g;
        this.draw(g2);
    }

    private void draw(Graphics2D g2) {
        for (RigidBody body : shapes) {
            g2.setColor(Color.WHITE);
            g2.draw(body.getPolygon());

            for (Vector2D point : body.getPoints()) {
                g2.drawRect((int)point.getX(), (int)point.getY(), 2, 2);
            }
        }
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        delay++;
        if (delay > 100) {
            
            for (RigidBody body : shapes) {
                body.update();
            }

            CD.detectCollision(this.shapes);
        }

        repaint();
    }
}