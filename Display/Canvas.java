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

    public Timer gameTimer = new Timer(1000/60, this);

    CollisionDetector CD;

    RigidBody square = new RigidBody(100, 450, 50, 50, 5);
    RigidBody rectangle = new RigidBody(320, 300, 100, 300, 6);

    public Canvas() {
        setBackground(new Color(50, 50, 50));
        gameTimer.start();

        shapes.add(square);
        shapes.add(rectangle);
        //shapes.get(1).applyForce(new Vector2D(200, 0), new Vector2D(0, -100));
        shapes.get(0).applyForce(new Vector2D(10, 0), new Vector2D(0, -2));
        //shapes.get(1).applyForce(new Vector2D(-5, 0), new Vector2D(0, 0));
        CD = new CollisionDetector(this.shapes);
    }

    @Override
    public void paint(Graphics g) {
        super.paint(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.translate(0, 0);
        this.draw(g2);
    }

    private void draw(Graphics2D g2) {
        for (RigidBody body : shapes) {
            g2.setColor(Color.WHITE);
            g2.draw(body.getPolygon());
        }
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        for (RigidBody body : shapes) {
            body.update();
        }
        CD.detectCollision(this.shapes);
        repaint();
    }
}