# Sphere Detection & Clustering

This project loads a point cloud from a PCD file, detects spheres using PCL, and groups them into *N* spatial clusters with OpenCV.

## Prerequisites

- **Operating System**: Ubuntu (including WSL2 on Windows)
- **C++ Compiler**: `g++` via `build-essential`
- **CMake** ≥ 3.5
- **Git**
- **PCL** (Point Cloud Library) development headers
- **OpenCV** development headers

Install dependencies on Ubuntu/WSL:

```bash
sudo apt update
sudo apt install -y build-essential cmake git libpcl-dev libopencv-dev
```

## Clone the Repository

```bash
git clone https://github.com/DreeN0T/pcl_sphere.git
cd pcl_sphere
```

## Build

1. Create and enter a build directory:
   ```bash
   mkdir build && cd build
   ```
2. Configure with CMake:
   ```bash
   cmake ../src
   ```
3. Compile the project:
   ```bash
   make -j$(nproc)
   ```

After successful build, the executable `pcl_sphere` will be in the `build/` folder.

## Run

Usage:

```bash
./pcl_sphere <path/to/input.pcd> <num_clusters>
```

- `<path/to/input.pcd>`: Path to your PCD file (relative or absolute)
- `<num_clusters>`: Number of clusters to group detected spheres into

Example:

```bash
./pcl_sphere ../data/data.pcd 3
```

Expected output:

```
Loaded point cloud with 150000 points.
Found sphere: center=(0.1, -0.2, 0.3) r=0.05, inliers=1200
Found sphere: center=(1.2, 0.7, -0.5) r=0.10, inliers=950
...  
Cluster 0:
  Sphere 0 - center=(0.1, -0.2, 0.3) r=0.05
Cluster 1:
  Sphere 1 - center=(1.2, 0.7, -0.5) r=0.10
```

## Troubleshooting

- **not found**: Ensure `libpcl-dev` is installed.
- **CMake errors**: Check that `find_package(PCL REQUIRED)` and `find_package(OpenCV REQUIRED)` succeed in the CMake output.
- **Permission denied**: Make the binary executable:
  ```bash
  chmod +x sphere_detection_clustering
  ```

---