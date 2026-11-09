FROM ros:jazzy-ros-base

ARG SMALL_GICP_VERSION=v1.0.0
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ninja-build ccache \
        gdb clang-format vim less sudo bash-completion \
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

# Non-root user matching the host UID/GID, so files written to the mounted repo
# keep the host owner. The base image's 'ubuntu' user is removed to free the IDs.
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && if ! getent group ${USER_GID} >/dev/null; then groupadd -g ${USER_GID} ${USERNAME}; fi \
    && useradd -m -s /bin/bash -u ${USER_UID} -g ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'export PS1="(degenbench) \[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ "' \
        >> /home/${USERNAME}/.bashrc

ENV CCACHE_DIR=/ws/.cache/ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache

USER ${USERNAME}
WORKDIR /ws
