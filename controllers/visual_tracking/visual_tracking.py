"""

Visual Tracking Controller fuer den NAO Roboter.

Ziel: eine gelbe Ente mit dem Kopf verfolgen und moeglichst genau im
Bildmittelpunkt halten. Der Score des Supervisors ist umso hoeher, je naeher
die Ente am Bildmittelpunkt liegt, wobei die horizontale Abweichung (x) staerker
gewichtet wird. Daher wird vor allem auf eine genaue Yaw Zentrierung geachtet.

Vorgehen:

1) Erkennung (Funktion detect):
   Das Kamerabild wird von BGRA nach BGR umgewandelt und in den HSV Farbraum
   ueberfuehrt. Mit einer Farbschwelle wird eine Maske fuer Gelb erzeugt, sodass
   die Ente von Feld, Robotern und Menschen getrennt wird.

2) Rauschunterdrueckung (Welt 3):
   In Welt 3 ist das Kamerabild verrauscht (cameraNoise). Ein GaussianBlur vor
   der Farbsegmentierung sowie ein medianBlur auf der Maske entfernen das
   Salt und Pepper Rauschen. Anschliessend werden mit morphologischem Opening
   Restflecken entfernt und mit Closing Loecher in der Ente geschlossen.

3) Blobauswahl:
   Aus den Konturen der Maske wird das groesste Blob gewaehlt, das eine
   Mindestflaeche besitzt und ein plausibles Seitenverhaeltnis hat. So werden
   Rauschartefakte und unpassende Formen verworfen. Der Schwerpunkt wird ueber
   die Bildmomente berechnet.

4) Regelung (P Regler):
   Aus der Abweichung des Schwerpunkts zur Bildmitte werden inkrementell die
   Sollwerte fuer HeadYaw und HeadPitch angepasst. Eine kleine Totzone verhindert
   Zittern, und die Gelenkwerte werden auf die physikalischen Grenzen des Halses
   begrenzt.

5) Zwei Kameras (Welt 4):
   In Welt 4 kommt die Ente nah an den Roboter und ist dann nur noch in der
   unteren Kamera sichtbar. Deshalb werden beide Kameras ausgewertet: zuerst die
   obere, und falls diese die Ente nicht sieht, die untere. Da die Vorzeichen der
   Abweichung fuer beide Kameras gleich sind, funktioniert derselbe Regler fuer
   beide.

6) Suchmodus:
   Wird die Ente in keiner Kamera gefunden, schwenkt der Kopf den Yaw Bereich ab,
   um sie wieder zu finden.
 
"""

import math
from controller import Robot, Camera  # type: ignore
import cv2
import numpy as np


robot = Robot()

# we process each image every 40ms = 25fps
timestep = int(robot.getBasicTimeStep() * 4)

# get access to the cameras and enable them
camera_top    = robot.getDevice("CameraTop")
camera_top.enable(timestep)

## enable the bottom camera if necessary
camera_bottom = robot.getDevice("CameraBottom")
camera_bottom.enable(timestep)

head_yaw   = robot.getDevice("HeadYaw")
head_pitch = robot.getDevice("HeadPitch")

# move arms down 
lShoulderPitch = robot.getDevice("LShoulderPitch")
rShoulderPitch = robot.getDevice("RShoulderPitch")
lShoulderPitch.setPosition( math.radians(90) )
rShoulderPitch.setPosition( math.radians(90) )

# Get the dimensions of the camera image
width = camera_top.getWidth()
height = camera_top.getHeight()

# P-Controller constants and initial target angles
Kp = 0.0004
target_yaw = 0.0
target_pitch = 0.0
search_direction = 1
search_speed = 0.02


def detect(camera):
    """Liefert (cx, cy) der gelben Ente in einem Kamerabild, sonst None."""
    raw_img = camera.getImage()
    if not raw_img:
        return None

    img = np.frombuffer(raw_img, np.uint8).reshape((height, width, 4))[:, :, [0, 1, 2]]

    # --- Welt 3: gegen das Sensorrauschen (cameraNoise) ---
    img = cv2.GaussianBlur(img, (3, 3), 0)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15, 120, 100])
    upper_yellow = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # medianBlur killt Salt&Pepper, OPEN entfernt Restflecken, CLOSE schließt die Ente
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < 60:                      # ferne/verrauschte Ente -> niedriger als 100
            continue
        x, y, w, h = cv2.boundingRect(c)
        ar = w / float(h) if h > 0 else 0
        if not (0.5 <= ar <= 1.8):         # leicht gelockert ggü. 0.75-1.35
            continue
        if area > best_area:
            best_area = area
            best = c

    if best is None:
        return None
    M = cv2.moments(best)
    if M["m00"] <= 0:
        return None
    return (M["m10"] / M["m00"], M["m01"] / M["m00"])

while robot.step(timestep) != -1:
    # Welt 4: erst obere Kamera, sonst untere (Ente nah am Roboter)
    result = detect(camera_top)
    if result is None:
        result = detect(camera_bottom)

    duck_found = result is not None
    if duck_found:
        cx, cy = result
        error_x = (width / 2) - cx
        error_y = cy - (height / 2)
        # Update motor positions using the Proportional Controller
        if abs(error_x) > 5:
            target_yaw += Kp * error_x
        if abs(error_y) > 5:
            target_pitch += Kp * error_y

    # If no duck was found, enter Search Mode
    if not duck_found:
        target_yaw += search_speed * search_direction
        if target_yaw >= 2.0:
            search_direction = -1
        elif target_yaw <= -2.0:
            search_direction = 1

    # Clamp the motor values to prevent exceeding NAO's neck physical limits
    target_yaw = max(-2.0, min(2.0, target_yaw))
    target_pitch = max(-0.6, min(0.5, target_pitch))

    # Apply the calculated positions to the motors
    head_yaw.setPosition(target_yaw)
    head_pitch.setPosition(target_pitch)
