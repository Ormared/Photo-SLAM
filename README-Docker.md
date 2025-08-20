# Photo-SLAM Docker Development Environment

This setup provides a containerized development environment for Photo-SLAM using Docker Compose with volume mounting for live development.

## Prerequisites

- Docker and Docker Compose installed
- NVIDIA Docker runtime installed
- NVIDIA GPU with CUDA support

## Quick Start

1. **Build and start the container:**
   ```bash
   docker compose up --build -d
   ```

2. **Enter the development container:**
   ```bash
   docker compose exec photo-slam bash
   ```

3. **Build Photo-SLAM inside the container:**
   ```bash
   # Inside the container
   ./build.sh
   ```

## Usage

### Development Workflow

The current directory is mounted as `/workspace` in the container, so any changes you make to the source code on your host system will be immediately reflected inside the container.

```bash
# Start the container
docker compose up -d

# Enter the container for development
docker compose exec photo-slam bash

# Your code changes are live-mounted, so you can edit on host
# and build/run inside the container
```

### Running Examples

```bash
# Inside the container
cd /workspace
./build.sh

# Run examples (adjust paths as needed)
./bin/replica_mono ./cfg/gaussian_mapper/Monocular/Replica/office0.yaml
```

```bash
./bin/replica_rgbd \
    ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
    ./cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
    ./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml \
    ./data/Replica/office0 \
    ./results
```


### Data and Results

- Mount your datasets to `/workspace/data`
- Results will be saved to `/workspace/results`
- Both directories are mounted as volumes for persistence

### GPU Support

The container is configured with NVIDIA GPU support. You can verify GPU access inside the container:

```bash
# Inside the container
nvidia-smi
```

## Container Management

```bash
# Start the container
docker-compose up -d

# Stop the container
docker-compose down

# View logs
docker-compose logs photo-slam

# Rebuild the container (after Dockerfile changes)
docker-compose up --build -d

# Remove container and volumes
docker-compose down -v
```

## GUI Applications

If you need to run GUI applications (like the viewer), make sure to:

1. Allow X11 forwarding on your host:
   ```bash
   xhost +local:docker
   ```

2. Set flags necessary to run GUI:
   ```bash
   env -u LIBGL_ALWAYS_INDIRECT -u LIBGL_ALWAYS_SOFTWARE -u GALLIUM_DRIVER -u MESA_GL_VERSION_OVERRIDE -u MESA_GLSL_VERSION_OVERRIDE ./bin/view_result ./cfg/view_only/view_only.yaml ./cfg/view_only/camera_replica.yaml ./results/6181_shutdown/ply/point_cloud/iteration_6181/point_cloud.ply

   ```

## File Structure

- `Dockerfile.dev`: Development Dockerfile (no git clone, uses volume mount)
- `docker-compose.yml`: Docker Compose configuration
- `.dockerignore`: Files to exclude from Docker build context
- `README-Docker.md`: This documentation

## Troubleshooting

### Permission Issues
If you encounter permission issues with mounted volumes:
```bash
# Run with your user ID
docker-compose exec --user $(id -u):$(id -g) photo-slam bash
```

### GPU Not Available
Ensure NVIDIA Docker runtime is properly installed:
```bash
# Test NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:11.8-base nvidia-smi
```
