import pybullet as p
import pybullet_data
import time
import os

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# Load the Kuka robots with a fixed base
kuka1 = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], useFixedBase=1)
kuka2 = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[1, 0, 0], baseOrientation=[0, 0, 0, 1], useFixedBase=1)

# Load the turntable
# tt_ws_path = os.environ.get('TT_WS_PATH', '/home/cam/misc_ws')
# turntable_urdf_path = os.path.join(tt_ws_path, 'src/tt_description/urdf/turn_table.urdf')
# turntable = p.loadURDF(turntable_urdf_path, basePosition=[0.5, 0.5, 0], baseOrientation=[0, 0, 0, 1], useFixedBase=1)
turntable = p.loadURDF('/home/cam/misc_ws/src/tt_description/urdf/turn_table.urdf', basePosition=[0.5, 0.5, 0], baseOrientation=[0, 0, 0, 1], useFixedBase=1)


# Create a battery to anchor the sheet
battery_position = [1, 1, 0.15]
battery = p.createMultiBody(
    baseMass=50,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.125, 0.05]),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.125, 0.05]),
    basePosition=battery_position
)

# Load and configure the soft body
sheet = p.loadSoftBody(
    "sheet_rough.obj",
    basePosition=[1, 1, 0.2],
    scale=0.01,
    mass=0.5,
    useNeoHookean=1,
    NeoHookeanMu=400,
    NeoHookeanLambda=600,
    NeoHookeanDamping=0.01,
    useSelfCollision=True,
    frictionCoeff=0.5,
    useFaceContact=True
)

if sheet < 0:
    print("Failed to load the soft body.")
else:
    num_nodes = p.getMeshData(sheet)[0]
    node_positions = p.getMeshData(sheet)[1]
    print("Number of nodes in soft body:", num_nodes)
    for i in range(num_nodes):
        pos = node_positions[i]
        # Add debug visualization to identify nodes
        p.addUserDebugLine(pos, [pos[0], pos[1], pos[2] + 0.05], [1, 0, 0], 0.01)

        # Anchor every fifth node as an example, adjust as needed
        if i % 5 == 0:
            try:
                p.createSoftBodyAnchor(sheet, nodeIndex=i, bodyUniqueId=battery, linkIndex=-1)
            except Exception as e:
                print(f"Failed to create anchor at node {i}: {str(e)}")

# Adjust physics engine parameters for stability
p.setPhysicsEngineParameter(numSolverIterations=150)

# Run the simulation
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()
