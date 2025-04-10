import open3d as o3d
import numpy as np
import math

def generate_equilateral_triangle_plate(r_b: float, plate_thickness: float) -> o3d.geometry.TriangleMesh:
    """
    Generate a thick equilateral triangle plate, circumscribed radius = r_b,
    with uniform thickness (plate_thickness), centered at origin, lying in XY plane.

    The triangle is extruded along z from -plate_thickness/2 to +plate_thickness/2.
    
    Returns an open3d.geometry.TriangleMesh
    """

    # Half-thickness above/below the XY plane
    half_t = plate_thickness * 0.5

    # We'll define 3 vertices in the XY plane with radius = r_b around the origin:
    # Standard equilateral triangle (120° apart):
    #   v0 at angle=0   => ( r_b, 0 )
    #   v1 at angle=120 => ( -r_b/2, +r_b*sqrt(3)/2 )
    #   v2 at angle=240 => ( -r_b/2, -r_b*sqrt(3)/2 )

    x0, y0 = r_b, 0
    x1, y1 = -0.5*r_b,  (math.sqrt(3)/2)*r_b
    x2, y2 = -0.5*r_b, -(math.sqrt(3)/2)*r_b

    # Top face (z = + half_t)
    top0 = np.array([x0, y0, +half_t], dtype=np.float32)
    top1 = np.array([x1, y1, +half_t], dtype=np.float32)
    top2 = np.array([x2, y2, +half_t], dtype=np.float32)

    # Bottom face (z = - half_t)
    bot0 = np.array([x0, y0, -half_t], dtype=np.float32)
    bot1 = np.array([x1, y1, -half_t], dtype=np.float32)
    bot2 = np.array([x2, y2, -half_t], dtype=np.float32)

    # Combine vertices: we order them [top0, top1, top2, bot0, bot1, bot2]
    vertices = np.vstack((top0, top1, top2,
                          bot0, bot1, bot2)).astype(np.float32)

    # Let's define each vertex index to keep code clear:
    TOP0 = 0
    TOP1 = 1
    TOP2 = 2
    BOT0 = 3
    BOT1 = 4
    BOT2 = 5

    # We'll define faces (triangles) for:
    #   1) top face  (0,1,2)
    #   2) bottom face (3,4,5) (we might want reversed order to ensure consistent normals)
    #   3) side faces:  each edge => 2 triangles

    faces = []

    # 1) top face => (0,1,2)
    faces.append([TOP0, TOP1, TOP2])
    # 2) bottom face => (BOT0,BOT2,BOT1) or some reversed order
    faces.append([BOT0, BOT2, BOT1])

    # 3) side faces:
    #   side0 => top0->top1->bot1->bot0
    #   side1 => top1->top2->bot2->bot1
    #   side2 => top2->top0->bot0->bot2

    # side0 => (0,1,4,3)
    # two triangles: (0,1,4), (0,4,3)
    faces.append([TOP0, TOP1, BOT1])
    faces.append([TOP0, BOT1, BOT0])

    # side1 => (1,2,5,4)
    faces.append([TOP1, TOP2, BOT2])
    faces.append([TOP1, BOT2, BOT1])

    # side2 => (2,0,3,5)
    faces.append([TOP2, TOP0, BOT0])
    faces.append([TOP2, BOT0, BOT2])

    faces = np.array(faces, dtype=np.int32)

    # Build open3d mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces)

    # Compute normals
    mesh.compute_vertex_normals()
    return mesh

if __name__ == "__main__":
    import sys

    # Example usage:
    # Equilateral triangle circumscribed radius=1.0, thickness=0.05
    r_b = 1.0
    base_plate_thickness = 0.05 * r_b
    
    r_e = 1.0
    end_plate_thickness = 0.05 * r_e

    mesh_b = generate_equilateral_triangle_plate(r_b, base_plate_thickness)
    mesh_e = generate_equilateral_triangle_plate(r_e, end_plate_thickness)

    # Save to stl
    filename_b = "base_plate.stl"
    o3d.io.write_triangle_mesh(filename_b, mesh_b)
    print(f"Wrote {filename_b}")
    
    filename_e = "end_plate.stl"
    o3d.io.write_triangle_mesh(filename_e, mesh_e)
    print(f"Wrote {filename_e}")

    # Also visualize with an axes frame
    # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
    # o3d.visualization.draw_geometries([mesh_b, axes])
