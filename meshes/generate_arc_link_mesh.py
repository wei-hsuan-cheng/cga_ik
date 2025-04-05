import open3d as o3d
import numpy as np
import math

def generate_partial_torus(
    center: np.ndarray,
    plane_normal: np.ndarray,
    start_dir: np.ndarray,
    torus_radius: float,
    tube_radius: float,
    start_deg: float,
    end_deg: float,
    steps_arc: int,
    steps_circle: int
) -> o3d.geometry.TriangleMesh:
    """
    Generate a partial torus (donut arc) by revolving a circle (tube_radius)
    around a main circle of radius torus_radius, from start_deg to end_deg in-plane.

    :param center:       (3,) The center of the torus circle
    :param plane_normal: (3,) Normal vector of the plane that contains the main torus circle
    :param start_dir:    (3,) A unit vector in-plane for angle = start_deg
    :param torus_radius: float, radius of the main circle (the “torus ring”)
    :param tube_radius:  float, radius of the cross-section circle
    :param start_deg:    float, start angle [deg] of the arc
    :param end_deg:      float, end angle [deg] of the arc
    :param steps_arc:    int, subdivision along the main arc
    :param steps_circle: int, subdivision around the cross-section
    :return:             open3d.geometry.TriangleMesh representing partial torus
    """

    # Convert angles to radians
    start_rad = math.radians(start_deg)
    end_rad   = math.radians(end_deg)

    # Normalize plane_normal and start_dir
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    start_dir    = start_dir / np.linalg.norm(start_dir)

    # In-plane perpendicular
    cross_dir = np.cross(plane_normal, start_dir)
    cross_dir /= np.linalg.norm(cross_dir)

    vertices = []
    faces = []

    # Param along arc: i in [0..steps_arc]
    arc_delta = (end_rad - start_rad) / steps_arc

    # For each point on the main arc, revolve a smaller circle of radius tube_radius
    # around the local tangent plane.
    for i in range(steps_arc + 1):
        arc_angle = start_rad + i*arc_delta
        cosA = math.cos(arc_angle)
        sinA = math.sin(arc_angle)
        # Center of cross-section:
        main_pt = center + torus_radius*(cosA*start_dir + sinA*cross_dir)

        # Main arc tangent direction
        arc_tangent = -sinA*start_dir + cosA*cross_dir
        arc_tangent /= np.linalg.norm(arc_tangent)

        # revolve_normal = arc_tangent x plane_normal
        revolve_normal = np.cross(arc_tangent, plane_normal)
        revolve_normal /= np.linalg.norm(revolve_normal)

        for j in range(steps_circle):
            theta = 2.0*math.pi * (j / steps_circle)
            cosT  = math.cos(theta)
            sinT  = math.sin(theta)
            offset = tube_radius*(cosT*revolve_normal + sinT*plane_normal)
            vertex = main_pt + offset
            vertices.append(vertex)

    vertices = np.array(vertices, dtype=np.float32)

    def vid(i, j):
        return i*steps_circle + j

    # Build faces
    for i in range(steps_arc):
        for j in range(steps_circle):
            j_next = (j+1) % steps_circle
            v0 = vid(i, j)
            v1 = vid(i, j_next)
            v2 = vid(i+1, j)
            v3 = vid(i+1, j_next)

            faces.append([v0, v1, v2])
            faces.append([v1, v3, v2])

    faces = np.array(faces, dtype=np.int32)

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()
    return mesh


if __name__ == "__main__":
    import sys

    # EXAMPLE:
    # partial torus from 0 to 90 [deg],
    # main circle radius=1.0, cross-section radius=0.1,
    # plane normal = z-axis => ring in XY plane
    # start_dir = x-axis => angle=0 along +X
    r_s = 1.0
    center     = np.array([0.0, 0.0, r_s], dtype=np.float32)
    plane_norm = np.array([0.0, -1.0, 0.0], dtype=np.float32)
    start_vec  = np.array([0.0, 0.0, -1.0], dtype=np.float32)

    mesh = generate_partial_torus(
        center       = center,
        plane_normal = plane_norm,
        start_dir    = start_vec,
        torus_radius = r_s,
        tube_radius  = r_s * 0.05,
        start_deg    = 0.0,
        end_deg      = 90.0,
        steps_arc    = 24,
        steps_circle = 24
    )

    # Save to ".stl"
    o3d.io.write_triangle_mesh("arc_link_thin.stl", mesh)
    print("Wrote arc_link_thin.stl")

    # Create a coordinate frame of size=0.5 at origin
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5,
        origin=[0, 0, 0]
    )

    # Visualize both the partial torus and the coordinate axes
    o3d.visualization.draw_geometries([mesh, axes])
