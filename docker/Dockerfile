FROM ros:jazzy-ros-base

ARG SMALL_GICP_VERSION=v1.0.0

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ninja-build \
        libeigen3-dev libgtest-dev \
        python3-pip python3-numpy python3-yaml python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# small_gicp: point-to-plane ICP and GICP solvers used by the labeler.
RUN git clone --depth 1 --branch ${SMALL_GICP_VERSION} \
        https://github.com/koide3/small_gicp.git /tmp/small_gicp \
    && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_HELPER=ON \
    && cmake --build /tmp/small_gicp/build \
    && cmake --install /tmp/small_gicp/build \
    && rm -rf /tmp/small_gicp \
    && ldconfig

WORKDIR /ws
