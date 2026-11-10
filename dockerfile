# =============================================================================
# Runtime image - uses pre-built builder image for fast iteration
# 
# First, build the builder image ONCE (takes ~1hr):
#   docker build -f dockerfile.builder -t blueos-oakd-basalt-builder .
#
# Then build runtime image quickly:
#   docker build -t blueos-oakd-basalt .
# =============================================================================
ARG BUILDER_IMAGE=blueos-oakd-basalt-builder

FROM ${BUILDER_IMAGE} AS builder

# =============================================================================
# Runtime stage - minimal image
# =============================================================================
FROM debian:trixie-slim

WORKDIR /app

# Install only runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-venv \
    libusb-1.0-0 libudev1 \
    libopencv-core410 libopencv-imgproc410 libopencv-imgcodecs410 \
    libboost-filesystem1.83.0 libboost-program-options1.83.0 \
    libgomp1 \
    tmux wget \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install ttyd
COPY install_ttyd.sh .
RUN ./install_ttyd.sh && rm install_ttyd.sh

# Copy built depthai Python bindings from builder
COPY --from=builder /app/depthai-core/build/bindings/python /app/depthai-python

# Copy ALL shared libraries from the build (depthai, etc.)
COPY --from=builder /app/depthai-core/build/*.so* /usr/local/lib/
# Copy vcpkg-built shared libraries (rtabmap, pcl, etc.)
COPY --from=builder /app/depthai-core/build/vcpkg_installed/arm64-linux/lib/*.so* /usr/local/lib/
# Copy FetchContent dependencies (dynamic_calibration, etc.)
COPY --from=builder /app/depthai-core/build/_deps/dynamic_calibration-src/lib/*.so* /usr/local/lib/
RUN ldconfig

# Create virtual environment and install Python deps
RUN python3 -m venv /app/venv
ENV PATH="/app/venv/bin:$PATH"
ENV PYTHONPATH="/app/depthai-python"

COPY requirements.txt /app/requirements.txt
RUN /app/venv/bin/pip install --no-cache-dir -r /app/requirements.txt

# Copy application code
COPY app /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# Labels
LABEL version="0.0.3"
LABEL permissions='{\
  "ExposedPorts": {"8080/tcp": {}},\
  "HostConfig": {\
    "NetworkMode": "host",\
    "Binds":["/dev/:/dev/", "/usr/blueos/extensions/oakd-vio:/app/data"],\
    "DeviceCgroupRules": [\
      "c 189:* rmw"\
    ]\
  }\
}'
LABEL authors='[{"name": "Willian Galvani", "email": "willian@bluerobotics.com"}]'
LABEL company='{"about": "", "name": "Blue Robotics", "email": "support@bluerobotics.com"}'
LABEL type="device-integration"
LABEL webpage='http://\{\{HOST\}\}:8080'
LABEL readme='https://raw.githubusercontent.com/williangalvani/Blueos-Oakd-Basalt/{tag}/README.md'
LABEL links='{"source": "https://github.com/williangalvani/Blueos-Oakd-Basalt"}'
LABEL requirements="core >= 1.1"

# Environment variables
ENV FPS=3
ENV CAMERA_ANGLE=0.0
ENV VEHICLE_IP=127.0.0.1

CMD ["/app/entrypoint.sh"]
