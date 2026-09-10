FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge \
    python3-opencv python3-numpy python3-scipy python3-yaml python3-pip \
    openssh-client iproute2 && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir --break-system-packages ruckig rich pillow
WORKDIR /app
COPY . /app
RUN install -m 0755 docker/pixi-shim.sh /usr/local/bin/pixi && \
    mkdir -p /root/.pixi/bin && ln -sf /usr/local/bin/pixi /root/.pixi/bin/pixi && \
    install -m 0755 docker/entrypoint.sh /entrypoint.sh && \
    printf 'Host *\n  StrictHostKeyChecking accept-new\n' > /etc/ssh/ssh_config.d/90-submission.conf
ENV PIXI=/usr/local/bin/pixi
ENTRYPOINT ["/entrypoint.sh"]
CMD ["run"]
