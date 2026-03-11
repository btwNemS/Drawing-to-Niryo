import tkinter as tk
import numpy as np
from pyniryo import *
from scipy.interpolate import splprep, splev

# -----------------------------
# CONFIG ROBOT
# -----------------------------
ROBOT_IP = "169.254.200.200"

DRAW_WIDTH_M = 0.35
DRAW_HEIGHT_M = DRAW_WIDTH_M * 0.5

Z_DRAW_OFFSET = 0.00
Z_LIFT_OFFSET = 0.03
Z_SAFE_MARGIN = 0.001

ARM_VELOCITY = 100
CHUNK_SIZE = 180

DRAW_ORIENTATION_JOINTS = JointsPosition(-0.0129, -0.6141, -0.6628, 0.1428, 1.2977, 1.5264)

SALUTATIONS = [
    JointsPosition(-0.0586, 0.1798, 0.5719, -0.0306, -0.3330, 1.6046),
    JointsPosition(-1.0996, -0.0172, -0.9310, -0.9663, -1.7626, 1.5049),
    JointsPosition(-0.0586, -0.0172, 0.2749, 0.0522, -0.7395, 1.4190),
    JointsPosition(0.4924, -0.1793, -0.9098, 0.9005, -1.6752, 1.2150),
    JointsPosition(-0.0586, 0.1798, 0.5719, -0.0306, -0.3330, 1.6046),
]

# -----------------------------
# CONFIG TKINTER
# -----------------------------
CANVAS_W = 1250
CANVAS_H = CANVAS_W * 0.5

MIN_PIXEL_STEP = 2.0
SMOOTHING_FACTOR = 1000.0
INTERPOLATION_POINTS = 150


# -----------------------------
# OUTILS
# -----------------------------
def spatial_downsample(stroke, min_dist_px=2.0):
    if min_dist_px <= 0 or len(stroke) < 2:
        return stroke

    filtered = [stroke[0]]
    last = np.array(stroke[0], dtype=float)

    for p in stroke[1:]:
        cur = np.array(p, dtype=float)
        if np.linalg.norm(cur - last) >= min_dist_px:
            filtered.append(p)
            last = cur

    if filtered[-1] != stroke[-1]:
        filtered.append(stroke[-1])

    return filtered


def smooth_stroke_spline(stroke, smoothing=2.0, n_points=150):
    if len(stroke) < 4:
        return stroke

    pts = np.array(stroke, dtype=float)
    x = pts[:, 0]
    y = pts[:, 1]

    distances = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)
    u = np.concatenate(([0], np.cumsum(distances)))
    if u[-1] == 0:
        return stroke

    u /= u[-1]

    tck, _ = splprep([x, y], u=u, s=smoothing)
    u_new = np.linspace(0, 1, n_points)
    x_new, y_new = splev(u_new, tck)

    return list(zip(x_new, y_new))


def pixel_to_robot(x_pix, y_pix, base_pose):
    dx = (x_pix - CANVAS_W / 2) / CANVAS_W * DRAW_WIDTH_M
    dy = (CANVAS_H / 2 - y_pix) / CANVAS_H * DRAW_HEIGHT_M

    x = base_pose.x + dx
    y = base_pose.y + dy

    x_rel = x - base_pose.x
    y_rel = y - base_pose.y

    x_rot = -y_rel
    y_rot = x_rel

    return base_pose.x + x_rot, base_pose.y + y_rot


def execute_in_chunks(robot, poses, chunk_size=180):
    for i in range(0, len(poses), chunk_size):
        robot.execute_trajectory(poses[i:i + chunk_size])


# -----------------------------
# APPLICATION
# -----------------------------
class DrawingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dessin lissé → Niryo Ned2")

        self.canvas = tk.Canvas(root, bg="white", width=CANVAS_W, height=CANVAS_H)
        self.canvas.pack()

        self.raw_strokes = []
        self.smoothed_strokes = []
        self.current_stroke = []

        self.canvas.bind("<Button-1>", self.start_stroke)
        self.canvas.bind("<B1-Motion>", self.draw_raw)
        self.canvas.bind("<ButtonRelease-1>", self.end_stroke)

        # Raccourcis clavier demandés
        self.root.bind("<Return>", self.on_enter_send)        # ENTRER -> envoyer au robot
        self.root.bind("<BackSpace>", self.on_backspace_reset)  # BACKSPACE -> reset
        self.root.bind("<space>", self.undo)                 # SPACE -> undo

        # Optionnel: garder Ctrl+Z
        self.root.bind("<Control-z>", self.undo)

        self.status = tk.StringVar(value="Prêt – Entrée: envoyer | Espace: undo | Backspace: reset")
        tk.Label(root, textvariable=self.status).pack(pady=5)

        # Pour que les touches fonctionnent tout de suite
        self.root.focus_set()

    # -----------------------------
    # HANDLERS CLAVIER
    # -----------------------------
    def on_enter_send(self, event=None):
        self.send_to_robot()
        return "break"  # évite des effets de bord Tk

    def on_backspace_reset(self, event=None):
        self.reset()
        return "break"

    # -----------------------------
    # DESSIN
    # -----------------------------
    def start_stroke(self, event):
        self.current_stroke = [(event.x, event.y)]

    def draw_raw(self, event):
        x, y = event.x, event.y
        self.current_stroke.append((x, y))

        self.canvas.create_line(
            self.current_stroke[-2][0],
            self.current_stroke[-2][1],
            x,
            y,
            fill="gray",
            dash=(2, 2)
        )

    def end_stroke(self, event):
        if len(self.current_stroke) < 2:
            return

        self.raw_strokes.append(self.current_stroke)

        stroke2 = spatial_downsample(self.current_stroke, MIN_PIXEL_STEP)
        stroke2 = smooth_stroke_spline(
            stroke2,
            smoothing=SMOOTHING_FACTOR,
            n_points=INTERPOLATION_POINTS
        )

        self.smoothed_strokes.append(stroke2)
        self.redraw_all()

    def redraw_all(self):
        self.canvas.delete("all")

        for stroke in self.smoothed_strokes:
            for i in range(1, len(stroke)):
                self.canvas.create_line(
                    stroke[i - 1][0],
                    stroke[i - 1][1],
                    stroke[i][0],
                    stroke[i][1],
                    fill="black",
                    width=2,
                    smooth=True
                )

    # -----------------------------
    # UNDO / RESET
    # -----------------------------
    def undo(self, event=None):
        if not self.smoothed_strokes:
            return "break"

        self.raw_strokes.pop()
        self.smoothed_strokes.pop()
        self.redraw_all()
        return "break"

    def reset(self):
        self.canvas.delete("all")
        self.raw_strokes = []
        self.smoothed_strokes = []
        self.current_stroke = []
        self.status.set("Reset – Prêt – Entrée: envoyer | Espace: undo | Backspace: reset")

    # -----------------------------
    # ROBOT
    # -----------------------------
    def send_to_robot(self):
        if not self.smoothed_strokes:
            self.status.set("Rien à envoyer")
            return

        self.status.set("Connexion robot…")
        self.root.update_idletasks()

        robot = None
        xr = yr = None  # pour le finally
        roll = pitch = yaw = None
        z_lift = None

        try:
            robot = NiryoRobot(ROBOT_IP)

            robot.update_tool()
            robot.calibrate_auto()
            robot.clear_collision_detected()

            base_pose = robot.forward_kinematics(DRAW_ORIENTATION_JOINTS)

            roll = base_pose.roll
            pitch = base_pose.pitch
            yaw = base_pose.yaw

            z_min_safe = base_pose.z - Z_SAFE_MARGIN
            z_draw = max(base_pose.z + Z_DRAW_OFFSET, z_min_safe)
            z_lift = max(base_pose.z + Z_LIFT_OFFSET, z_min_safe)

            robot.set_arm_max_velocity(ARM_VELOCITY)

            for stroke in self.smoothed_strokes:
                x0, y0 = pixel_to_robot(stroke[0][0], stroke[0][1], base_pose)

                robot.move_pose(PoseObject(x0, y0, z_lift, roll, pitch, yaw))
                robot.move_pose(PoseObject(x0, y0, z_draw, roll, pitch, yaw))

                poses = []
                for (x_pix, y_pix) in stroke:
                    xr, yr = pixel_to_robot(x_pix, y_pix, base_pose)
                    poses.append(PoseObject(xr, yr, z_draw, roll, pitch, yaw))

                execute_in_chunks(robot, poses, CHUNK_SIZE)

                robot.move_pose(PoseObject(xr, yr, z_lift, roll, pitch, yaw))

            self.status.set("Terminé (Entrée pour relancer)")

        except Exception as e:
            self.status.set(f"Erreur: {e}")

        finally:
            if robot:
                try:
                    # Relever le stylo à la dernière position connue (si dispo)
                    if xr is not None and yr is not None and z_lift is not None:
                        robot.move_pose(PoseObject(xr, yr, z_lift, roll, pitch, yaw))
                    for joints in SALUTATIONS:
                        robot.move(joints)
                finally:
                    robot.close_connection()


# -----------------------------
# MAIN
# -----------------------------
root = tk.Tk()  
app = DrawingApp(root)
root.mainloop()