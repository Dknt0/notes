Blender Note
===

> To record some basic operations used in Blender. This document is based on Blender 4.2.3 LTS.
>
> Dknt 2025.1.21

# Optimize Mesh File for URDF Model

**Convex Hull**. Import the STL file. Select the object in *Object Mode*, then press Tab to switch to *Edit Mode*. Press `A` to select the whole mesh. Select *Mesh > Convex Hull* in the top menu (shortcut: `Ctrl + Shift + Alt + C`).

**Merge Vertices**. Press `A` to select all vertices. Press `Alt + M` to merge vertices. Select *By Distance*. Then the mesh is optimized. Press `Alt + J` to convert the mesh to quads.
he Convex Hull Modifier
**Reduce Face**. Click the mesh. Then check the *Modifiers* panel. Add a *Decimate* modifier. Set a *Ratio*. Apply the modifier `Ctrl + A`. Export the mesh as STL file.

**Generate Face**. In *Edit Mode*, selecting vertices, then press F to generate a face.

**Separate Mesh**. Press `B` for box selection, `C` for circle selection, `L` to select linked vertices. Press `P` and choose *Selection* to separate the mesh. Press `Ctrl + J` to join the mesh.







