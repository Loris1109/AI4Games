package s0591904;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Path2D;
import java.util.Arrays;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;


public class MyAi extends AI {

    private Point[] pearl;
    private Path2D[] obstacles;
    float[] target;
    float[] avoidTarget;
    float[] predictionPoint;
    boolean avoiding = false;
    int currentTargetIndex = 0;
    private final float maxAngularVelocity = (float) Math.toRadians(10f);
    private final float maxAngularAcceleration = (float) Math.toRadians(5f);


    public MyAi(Info info) {
        super(info);
        enlistForTournament(591904);
        pearl = info.getScene().getPearl();
        obstacles = info.getScene().getObstacles();
    }

    @Override
    public String getName() {
        return "Lorenz";
    }

    @Override
    public Color getPrimaryColor() {
        return Color.BLUE;
    }

    @Override
    public Color getSecondaryColor() {
        return new Color(0, 255, 0);
    }

    @Override
    public PlayerAction update() {
        predictionPoint = PredictionAhead();
        float avoidancePower = AvoidObstacles(predictionPoint);
        float power = 0;
        float angularAcceleration = 0;


        if(currentTargetIndex == pearl.length) return new DivingAction(0,0);

        target = new float[]{
                pearl[currentTargetIndex].x,
                pearl[currentTargetIndex].y};
        float distance = distanceTo(target);
        if (distance < 1f) {
            currentTargetIndex++;
        }

        if(avoiding)
        {
            // Move sideways instead of facing the pearl
            float sideOffset = 25; // adjust if needed

            float[] avoidanceDirection = new float[] {
                    -(float) Math.sin(info.getOrientation()),  // this gives you a perpendicular vector
                    (float) Math.cos(info.getOrientation())
            };

            avoidTarget = new float[]{
                    info.getX() + avoidanceDirection[0] * sideOffset,
                    info.getY() - avoidanceDirection[1] * sideOffset
            };

            angularAcceleration = Align(avoidTarget, (float) Math.toRadians(5), (float) Math.toRadians(30), 0.5f);
            power = avoidancePower;
        }
        else
        {
            power = Arrive(target, 1.0f, 5.0f, 0.1f);
            angularAcceleration = Align(target, (float) Math.toRadians(5), (float) Math.toRadians(30), 0.5f);
        }
        System.out.println(power);


        return new DivingAction(power, angularAcceleration);
    }

    private float distanceTo(float[] target) {
        float dx = target[0] - info.getX();
        float dy = target[1] - info.getY();
        return (float) Math.sqrt(dx*dx + dy*dy);
    }

    private float[] PredictionAhead()
    {
        //predict where I am going
        float orientation = info.getOrientation();
        float speed = info.getVelocity() * 50;
        float[] ahead = new float[]{
                info.getX() + (float)Math.cos(orientation) * speed,
                info.getY() - (float)Math.sin(orientation) * speed
        };
        return ahead;
    }

    private float AvoidObstacles(float[] predictionPoint)
    {
        for (Path2D obs : obstacles) {
            // Create a small rectangle around the prediction point to check for intersection
            float checkSize = 5f; // Adjust based on how "thick" you want the collision detection
            if (obs.intersects(predictionPoint[0] - checkSize / 2, predictionPoint[1] - checkSize / 2, checkSize, checkSize))
            {
                // Obstacle detected — apply perpendicular avoidance force
                // Compute perpendicular direction to the current orientation
                float[] perpendicular = new float[]{
                        -(float) Math.sin(info.getOrientation()),
                        (float) Math.cos(info.getOrientation())
                };

                // Calculate the relative position of the obstacle to the player
                float[] obstacleVector = new float[]{
                        (float) (obs.getBounds2D().getCenterX() - info.getX()),
                        (float) (obs.getBounds2D().getCenterY() - info.getY())
                };

                // Calculate the cross product to determine which side the obstacle is on
                float cross = perpendicular[0] * obstacleVector[1] - perpendicular[1] * obstacleVector[0];
                float directionSign = Math.signum(cross); // +1: left, -1: right

                // Apply maximum acceleration in the perpendicular direction
                float[] avoidAcceleration = new float[]{
                        perpendicular[0] * info.getMaxAcceleration(),
                        perpendicular[1] * info.getMaxAcceleration()
                };
                avoidAcceleration[0] *= directionSign;
                avoidAcceleration[1] *= directionSign;
                float magnitude = (float) Math.sqrt(avoidAcceleration[0] * avoidAcceleration[0] + avoidAcceleration[1] * avoidAcceleration[1]);


                float power = magnitude;

                avoiding = true;
                return power;

            }

        }
        avoiding = false;
        return 0;
    }

    private float[] Seek(float[] targetPos)
    {
        float[] direction = new float[] {targetPos[0]-info.getX(),targetPos[1]-info.getY()};
        float distance = (float) Math.sqrt(Math.pow(direction[0],2) + Math.pow(direction[1],2));
        float[] normDirection = new float[] {direction[0]/distance, direction[1]/distance};
        float[] acceleration = new float[]{normDirection[0]*info.getMaxAcceleration(), normDirection[1]*info.getMaxAcceleration()};
        return acceleration;
    }


    private float VelocityMatching(float targetSpeed, float currentSpeed, float timeForAction)
    {
        float acceleration = (targetSpeed - currentSpeed)/timeForAction;
        acceleration = Math.min(Math.max(acceleration, 0f), info.getMaxAcceleration());
        return acceleration;
    }

    private float Align(float[] targetPos, float alignToleranz, float slowDownAngle, float timeToTarget) {
        float[] direction = new float[]{targetPos[0] - info.getX(), targetPos[1] - info.getY()};
        float targetOrientation = (float) Math.atan2(-direction[1], direction[0]);
        // kleinster Winkel zwischen 2 Orientierungen
        float rotation = normalizeAngle(targetOrientation - info.getOrientation()); //richtung
        float rotationSize = Math.abs(rotation); //winkelbetrag = wie viel noch

        if (rotationSize < alignToleranz) {
            return 0; //arrived
        }

        // Ziel-Drehgeschwindigkeit berechnen
        float targetAngularVelocity;
        if (rotationSize <= slowDownAngle) {
            targetAngularVelocity = rotationSize * maxAngularVelocity / slowDownAngle;
        } else {
            targetAngularVelocity = maxAngularVelocity;
        }

        // Richtung berücksichtigen
        targetAngularVelocity *= Math.signum(rotation);

        // Wunsch-Drehbeschleunigung
        float angularAcceleration = (targetAngularVelocity - info.getAngularVelocity()) / timeToTarget;

        if (Math.abs(angularAcceleration) > maxAngularAcceleration) {
            angularAcceleration = maxAngularAcceleration * Math.signum(angularAcceleration);
        } //Begrenzung

        return angularAcceleration;

    }

    // Gibt einen Winkel zurück im Bereich [-π, +π]
    float normalizeAngle(float angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private float Arrive (float[] target, float targetRadius, float slowDownRadius, float timeToTarget )
    {
        //Richtung & Abstand berechnen
        float[] direction = new float[] {target[0]-info.getX(),target[1]-info.getY()};
        float distance = (float) Math.sqrt(Math.pow(direction[0],2) + Math.pow(direction[1],2));
        // Ziel erreicht → keine Bewegung
        if(distance < targetRadius)
            return 0;
        //Wunschgeschwindigkeit bestimmen
        float targetSpeed;
        if(distance < slowDownRadius)
        {
            targetSpeed = info.getMaxVelocity() * (distance/slowDownRadius);
        }
        else
        {
            targetSpeed = info.getMaxVelocity();
        }

        //Wunschgeschwindigkeit = Richtung * Zielgeschwindigkeit
        float[] desiredVelocity = new float[] {direction[0]/distance * targetSpeed, direction[1]/distance * targetSpeed};
        float velocityX = (float) (Math.cos(info.getOrientation()) * info.getVelocity());
        float velocityY = (float) (Math.sin(info.getOrientation()) * info.getVelocity());

        //Beschleunigung berechnen
        float[] desiredAcceleration = new float[] {
                desiredVelocity[0]-velocityX * (1/timeToTarget),
                desiredVelocity[1]-velocityY * (1/timeToTarget)
        };
        float accMagnitude = (float) Math.sqrt(Math.pow(desiredAcceleration[0],2) + Math.pow(desiredAcceleration[1],2));
        if (accMagnitude > info.getMaxAcceleration()) {
            desiredAcceleration[0] = desiredAcceleration[0] / accMagnitude * info.getMaxAcceleration();
            desiredAcceleration[1] = desiredAcceleration[1] / accMagnitude * info.getMaxAcceleration();
            accMagnitude = info.getMaxAcceleration(); // reset it
        }

        float[] facing = new float[]{
                (float) Math.cos(info.getOrientation()),
                (float) Math.sin(info.getOrientation())};
        // Direction to target (normalized)
        float[] desiredDir = new float[]{direction[0] / distance, direction[1] / distance};
        float dot = facing[0] * desiredDir[0] + facing[1] * desiredDir[1];
        dot = Math.max(-1, Math.min(1, dot));
        float angleBetween = (float) Math.acos(dot);

        float angleTolerance = (float) Math.toRadians(22);
        float alignment = (angleBetween < angleTolerance) ? 1f : Math.max(0f, dot);

        // Final power = acceleration * alignment factor
        float power = accMagnitude * alignment;
        return power;
    }


    @Override
    public void drawDebugStuff(Graphics2D gfx) {
        int i =1;
        for (Point p : pearl) {
            gfx.setColor(Color.BLACK);
            gfx.drawOval((int)p.getX(), (int)p.getY(), 20, 20);
            gfx.drawString(String.valueOf(i), p.x, p.y);
            i++;
        }
        gfx.setColor(Color.RED);
        gfx.drawLine((int) info.getX(), (int) info.getY(), (int) target[0], (int) target[1]);
        gfx.setColor(Color.MAGENTA);
        if(predictionPoint != null)
        {
            gfx.drawOval((int) predictionPoint[0], (int) predictionPoint[1],1,1);
            gfx.drawRect((int) (predictionPoint[0] - 5 / 2), (int) (predictionPoint[1] - 5 / 2), 5, 5);
        }
        if(avoidTarget != null)
        {
            gfx.setColor(Color.GREEN);
            gfx.drawRect((int) (avoidTarget[0] - 5 / 2), (int) (avoidTarget[1] - 5 / 2), 5, 5);
        }

    }
}
