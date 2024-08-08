import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as Axes3D

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# box_position =

# Sheet dimensions and properties
sheet_length = 0.3  # length in meters
sheet_width = 0.25   # width in meters
thickness = 0.0015  # thickness in meters
density = 2700      # density of aluminum in kg/m³

# Number of nodes (resolution of the mesh)
num_nodes_length = 10
num_nodes_width = 10

# Calculate area and volume per node
node_area = (sheet_length / num_nodes_length) * (sheet_width / num_nodes_width)
node_volume = node_area * thickness
node_mass = node_volume * density

# Stiffness estimation based on material properties (may need adjustment)
youngs_modulus = 69e9  # Young's modulus of aluminum in Pascals
stiffness = youngs_modulus * node_area * thickness

# Soft body parameters (these may need tuning)
nodeRadius = 0.1
bendingStiffness = 0.1
damping = 0.01

# Create soft body
sheetId = p.loadSoftBody("untitled.obj", basePosition=[0,0,0], scale=0.001, mass=node_mass, useNeoHookean=1, NeoHookeanMu=stiffness, NeoHookeanLambda=stiffness, NeoHookeanDamping=damping, useSelfCollision=1, frictionCoeff=0.5, repulsionStiffness=800)
num_of_nodes = p.getMeshData(sheetId)[0]
print("Nodes = ", num_of_nodes)
node_positions = p.getMeshData(sheetId)[1]
print(node_positions)
x_coords = [pos[0] for pos in node_positions]
y_coords = [pos[1] for pos in node_positions]
z_coords = [pos[2] for pos in node_positions]

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # Scatter plot
# scatter = ax.scatter(x_coords, y_coords, z_coords, c='r', marker='o')
#
# # for i, (x, y, z) in enumerate(zip(x_coords, y_coords, z_coords)):
# #     ax.text(x, y, z, f'{i}', color='blue', size=7)
#
# # Adding labels
# ax.set_xlabel('X Coordinate')
# ax.set_ylabel('Y Coordinate')
# ax.set_zlabel('Z Coordinate')
#
# # Title
# ax.set_title('3D Scatter Plot of Node Positions')
#
# # Show the plot
# plt.show()

target_position = [0.5, 0.5, 0]
corner_node_index = 0
p.createSoftBodyAnchor(sheetId, corner_node_index, -1, -1, target_position)

# Simulation loop
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)
