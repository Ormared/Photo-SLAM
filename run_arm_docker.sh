#!/bin/bash

# Photo-SLAM ARM Docker Container Runner
# Builds and runs the ARM Docker container for Photo-SLAM

set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
IMAGE_NAME="photo-slam-arm"
CONTAINER_NAME="photo-slam-arm-container"

# Docker arguments array
DOCKER_ARGS=(
    "--gpus=all"
    "--device=/dev/dri"
    "--env=DISPLAY=$DISPLAY"
    "--env=QT_X11_NO_MITSHM=1"
    "--env=XAUTHORITY=$XAUTHORITY"
    "-v /tmp/.X11-unix:/tmp/.X11-unix:rw"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to build the Docker image
build_image() {
    print_info "Building Docker image: $IMAGE_NAME"
    
    if ! docker build -f Dockerfile.arm -t "$IMAGE_NAME" "$WORKSPACE_DIR"; then
        print_error "Failed to build Docker image"
        exit 1
    fi
    
    print_info "Docker image built successfully: $IMAGE_NAME"
}

# Function to run the container
run_container() {
    print_info "Starting container: $CONTAINER_NAME"
    
    # Check if container already exists and remove it
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        print_warning "Container $CONTAINER_NAME already exists. Removing..."
        docker rm -f "$CONTAINER_NAME"
    fi
    
    # Create data directory if it doesn't exist
    mkdir -p "$WORKSPACE_DIR/data"
    mkdir -p "$WORKSPACE_DIR/results"
    
    print_info "Running Docker container with the following setup:"
    echo "  - Image: $IMAGE_NAME"
    echo "  - Container: $CONTAINER_NAME"
    echo "  - Workspace: $WORKSPACE_DIR"
    echo "  - GPU support: enabled"
    echo "  - Display forwarding: enabled"
    
    docker run -it --rm \
        --privileged \
        --network host \
        --ipc=host \
        "${DOCKER_ARGS[@]}" \
        -v "$WORKSPACE_DIR:/workspace/photo-slam" \
        -v "$WORKSPACE_DIR/data:/workspace/data" \
        -v "$WORKSPACE_DIR/results:/workspace/results" \
        -v /etc/localtime:/etc/localtime:ro \
        --name "$CONTAINER_NAME" \
        --runtime nvidia \
        --workdir /workspace/photo-slam \
        "$IMAGE_NAME" \
        /bin/bash
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  build           Build the Docker image only"
    echo "  run             Run the container (builds image if needed)"
    echo "  rebuild         Force rebuild the image and run"
    echo "  --help, -h      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build        # Build the image only"
    echo "  $0 run          # Build (if needed) and run the container"
    echo "  $0 rebuild      # Force rebuild and run"
}

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    print_error "Docker is not running or not accessible"
    echo "Please start Docker and ensure you have proper permissions"
    exit 1
fi

# Check for NVIDIA Docker runtime
if ! docker info | grep -q nvidia; then
    print_warning "NVIDIA Docker runtime not detected"
    print_warning "GPU acceleration may not be available"
fi

# Parse command line arguments
case "${1:-run}" in
    "build")
        build_image
        ;;
    "run")
        # Check if image exists, build if not
        if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
            print_info "Image $IMAGE_NAME not found. Building..."
            build_image
        fi
        run_container
        ;;
    "rebuild")
        print_info "Force rebuilding image..."
        build_image
        run_container
        ;;
    "--help"|"-h"|"help")
        show_usage
        ;;
    *)
        print_error "Unknown option: $1"
        show_usage
        exit 1
        ;;
esac