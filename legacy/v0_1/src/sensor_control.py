import pybullet as p
import pybullet_data
import time
import random


# ==========================================
# 1. N7ell simulation
# ==========================================

p.connect(p.GUI)

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)


# ==========================================
# 2. Ndakhlou l2ard
# ==========================================

plane_id = p.loadURDF(
    "plane.urdf"
)


# ==========================================
# 3. Ndakhlou obstacle
# ==========================================

obstacle_id = p.loadURDF(
    "world/obstacle.urdf",
    basePosition=[0.90, 0, 0],
    useFixedBase=True
)


# ==========================================
# 4. Ndakhlou RoverGuard
# ==========================================

robot_id = p.loadURDF(
    "robot/roverguard.urdf",
    basePosition=[0, 0, 0.15]
)


# ==========================================
# 5. N9raw joints dyal robot
# ==========================================

number_of_joints = p.getNumJoints(
    robot_id
)

print(
    "Nombre de joints:",
    number_of_joints
)

joint_indices = {}

for joint_index in range(number_of_joints):

    joint_info = p.getJointInfo(
        robot_id,
        joint_index
    )

    joint_name = joint_info[1].decode(
        "utf-8"
    )

    link_name = joint_info[12].decode(
        "utf-8"
    )

    joint_indices[joint_name] = joint_index

    print(
        "Index:", joint_index,
        "| Joint:", joint_name,
        "| Link:", link_name
    )


# ==========================================
# 6. Nakhdo index dyal kol joint
# ==========================================

left_joint = joint_indices[
    "left_wheel_joint"
]

right_joint = joint_indices[
    "right_wheel_joint"
]

caster_joint = joint_indices[
    "rear_caster_joint"
]

sensor_joint = joint_indices[
    "front_sensor_joint"
]


# ==========================================
# 7. Friction
# ==========================================

p.changeDynamics(
    robot_id,
    left_joint,
    lateralFriction=1.2,
    rollingFriction=0.001
)

p.changeDynamics(
    robot_id,
    right_joint,
    lateralFriction=1.2,
    rollingFriction=0.001
)

# Friction 9lila f kora lwer
p.changeDynamics(
    robot_id,
    caster_joint,
    lateralFriction=0.01,
    rollingFriction=0.001
)


# ==========================================
# 8. Camera
# ==========================================

p.configureDebugVisualizer(
    p.COV_ENABLE_GUI,
    0
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[
        0.25,
        0,
        0.05
    ]
)


# ==========================================
# 9. Nkhalliw robot ysta9er
# ==========================================

for step in range(240):
    p.stepSimulation()
    time.sleep(1 / 240)


# ==========================================
# 10. Settings dyal sensor
# ==========================================

MAX_DISTANCE = 1.0

STOP_DISTANCE = 0.25
CLEAR_DISTANCE = 0.30

NOISE_STD = 0.02
FILTER_ALPHA = 0.20

filtered_distance = None

debug_line_id = -1
safety_blocked = False

counter = 0
running = True


print("")
print("Fleches = t7errek")
print("Q = khroj")
print("STOP f 25 cm")
print("PATH CLEAR f 30 cm")
print("")


# ==========================================
# 11. Loop l2asasiya
# ==========================================

try:

    while running and p.isConnected():

        # F kol doura kanbdaw b zero
        forward_speed = 0.0
        turn_speed = 0.0


        # ==================================
        # 12. N9raw clavier
        # ==================================

        keys = p.getKeyboardEvents()

        if (
            p.B3G_UP_ARROW in keys
            and keys[p.B3G_UP_ARROW]
            & p.KEY_IS_DOWN
        ):
            forward_speed = 5.0

        if (
            p.B3G_DOWN_ARROW in keys
            and keys[p.B3G_DOWN_ARROW]
            & p.KEY_IS_DOWN
        ):
            forward_speed = -5.0

        if (
            p.B3G_LEFT_ARROW in keys
            and keys[p.B3G_LEFT_ARROW]
            & p.KEY_IS_DOWN
        ):
            turn_speed = 3.0

        if (
            p.B3G_RIGHT_ARROW in keys
            and keys[p.B3G_RIGHT_ARROW]
            & p.KEY_IS_DOWN
        ):
            turn_speed = -3.0

        if ord("q") in keys:
            if (
                keys[ord("q")]
                & p.KEY_WAS_TRIGGERED
            ):
                running = False
                break


        # ==================================
        # 13. Position dyal sensor
        # ==================================

        sensor_state = p.getLinkState(
            robot_id,
            sensor_joint,
            computeForwardKinematics=True
        )

        sensor_position = sensor_state[0]
        sensor_orientation = sensor_state[1]


        # ==================================
        # 14. Jiha l9dam dyal sensor
        # ==================================

        rotation = p.getMatrixFromQuaternion(
            sensor_orientation
        )

        forward_vector = [
            rotation[0],
            rotation[3],
            rotation[6]
        ]


        # ==================================
        # 15. Bidayat ray
        # ==================================

        ray_start = [
            sensor_position[0]
            + forward_vector[0] * 0.03,

            sensor_position[1]
            + forward_vector[1] * 0.03,

            sensor_position[2]
            + forward_vector[2] * 0.03
        ]


        # ==================================
        # 16. Nihayat ray
        # ==================================

        ray_end = [
            ray_start[0]
            + forward_vector[0]
            * MAX_DISTANCE,

            ray_start[1]
            + forward_vector[1]
            * MAX_DISTANCE,

            ray_start[2]
            + forward_vector[2]
            * MAX_DISTANCE
        ]


        # ==================================
        # 17. Ray test
        # ==================================

        ray_result = p.rayTestBatch(
            [ray_start],
            [ray_end]
        )[0]

        hit_object_id = ray_result[0]
        hit_fraction = ray_result[2]


        # ==================================
        # 18. True distance
        # ==================================

        if hit_object_id == -1:

            true_distance = MAX_DISTANCE
            line_end = ray_end

        else:

            true_distance = (
                hit_fraction
                * MAX_DISTANCE
            )

            line_end = ray_result[3]


        # ==================================
        # 19. Noise
        # ==================================

        noise = random.gauss(
            0.0,
            NOISE_STD
        )

        raw_distance = (
            true_distance + noise
        )


        # Nkhalliw l9yass bin 0 w 1 m
        raw_distance = max(
            0.0,
            min(
                MAX_DISTANCE,
                raw_distance
            )
        )


        # ==================================
        # 20. Filter
        # ==================================

        if filtered_distance is None:

            filtered_distance = (
                raw_distance
            )

        else:

            filtered_distance = (
                FILTER_ALPHA
                * raw_distance

                + (1.0 - FILTER_ALPHA)
                * filtered_distance
            )


        # Controller kaysta3mel filter
        distance = filtered_distance


        # ==================================
        # 21. Safety b hysteresis
        # ==================================

        if (
            not safety_blocked
            and distance < STOP_DISTANCE
        ):

            safety_blocked = True

            print(
                "\nSTOP: obstacle 9rib!"
            )

        elif (
            safety_blocked
            and distance > CLEAR_DISTANCE
        ):

            safety_blocked = False

            print(
                "\nPATH CLEAR: robot y9der ymchi"
            )


        # Ila safety blocked,
        # manmchiwch l9dam
        if (
            safety_blocked
            and forward_speed > 0
        ):

            forward_speed = 0.0


        # ==================================
        # 22. Loun dyal ray
        # ==================================

        if safety_blocked:

            line_color = [1, 0, 0]

        else:

            line_color = [0, 1, 0]


        debug_line_id = (
            p.addUserDebugLine(
                ray_start,
                line_end,
                line_color,
                lineWidth=3,
                replaceItemUniqueId=(
                    debug_line_id
                )
            )
        )


        # ==================================
        # 23. Differential drive
        # ==================================

        left_speed = (
            forward_speed - turn_speed
        )

        right_speed = (
            forward_speed + turn_speed
        )


        # ==================================
        # 24. Moteur roue lisra
        # ==================================

        p.setJointMotorControl2(
            robot_id,
            left_joint,
            p.VELOCITY_CONTROL,
            targetVelocity=left_speed,
            force=2.0
        )


        # ==================================
        # 25. Moteur roue limna
        # ==================================

        p.setJointMotorControl2(
            robot_id,
            right_joint,
            p.VELOCITY_CONTROL,
            targetVelocity=right_speed,
            force=2.0
        )


        # ==================================
        # 26. N9eddmo simulation
        # ==================================

        p.stepSimulation()
        time.sleep(1 / 240)

        counter += 1


        # Ntb3o l9yass 4 mrrat f seconde
        if counter % 60 == 0:

            print(
                f"\rTrue: "
                f"{true_distance:.2f} m"

                f" | Raw: "
                f"{raw_distance:.2f} m"

                f" | Filter: "
                f"{filtered_distance:.2f} m",

                end=""
            )


# Ila fenêtre tsedat b X
except p.error:

    print(
        "\nFenetre PyBullet tsedat."
    )


# ==========================================
# 27. Nihayat simulation
# ==========================================

print("")

if p.isConnected():

    final_position, _ = (
        p.getBasePositionAndOrientation(
            robot_id
        )
    )

    print(
        "Position finale:",
        final_position
    )

    p.disconnect()

else:

    print(
        "PyBullet tsed bla traceback."
    )