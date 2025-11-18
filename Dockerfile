FROM ubuntu:16.04

# 控制并行编译线程数，避免在低内存环境下 OOM
ARG MAKE_JOBS=2
ARG CMAKE_VERSION=3.22.6

# 避免交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# 基础依赖与证书
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ca-certificates wget git pkg-config \
      build-essential \
      libpcl-dev libeigen3-dev libboost-all-dev libflann-dev libvtk6-dev \
      libmysqlclient-dev && \
    rm -rf /var/lib/apt/lists/*

# 安装较新版本 CMake（Ubuntu16 默认太旧），按架构选择正确的包
RUN set -eux; \
    arch="$(uname -m)"; \
    if [ "$arch" = "x86_64" ]; then \
      wget -q "https://cmake.org/files/v3.22/cmake-${CMAKE_VERSION}-linux-x86_64.sh" -O /tmp/cmake.sh; \
      chmod +x /tmp/cmake.sh; \
      /tmp/cmake.sh --prefix=/usr/local --skip-license; \
      rm -f /tmp/cmake.sh; \
    elif [ "$arch" = "aarch64" ] || [ "$arch" = "arm64" ]; then \
      wget -q "https://cmake.org/files/v3.22/cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz" -O /tmp/cmake.tar.gz; \
      mkdir -p /opt; \
      tar -xf /tmp/cmake.tar.gz -C /opt; \
      ln -sf /opt/cmake-${CMAKE_VERSION}-linux-aarch64/bin/* /usr/local/bin/; \
      rm -f /tmp/cmake.tar.gz; \
    else \
      apt-get update && apt-get install -y --no-install-recommends cmake && rm -rf /var/lib/apt/lists/*; \
    fi

# 安装 Draco 并导出 draco::draco 包给 CMake
RUN git clone --depth 1 https://github.com/google/draco.git /tmp/draco && \
    mkdir -p /tmp/draco/build && \
    cd /tmp/draco/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=ON .. && \
    make -j"$MAKE_JOBS" && make install && ldconfig && \
    rm -rf /tmp/draco

WORKDIR /app

# 复制源码（使用 .dockerignore 控制上下文）
COPY . /app

# 预置配置到工作目录，程序用相对路径读取
RUN cp -f /app/persist/mysql.ini /app/mysql.ini

# 构建
RUN mkdir -p /app/build && \
    cd /app/build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j"$MAKE_JOBS"

# 入口脚本负责修正配置并以前台模式启动
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8080

CMD ["/entrypoint.sh"]