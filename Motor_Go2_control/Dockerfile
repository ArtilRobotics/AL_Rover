FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    libserialport-dev \
    libspnav-dev \
    libx11-dev \
    libxi-dev \
    libxcursor-dev \
    libflac-dev \
    libogg-dev \
    libvorbis-dev \
    libopenal-dev \
    libfreetype6-dev \
    libxrandr-dev \
    libudev-dev \
    libgl1-mesa-dev \
    libjpeg-dev \
    libxcb1-dev \
    libpthread-stubs0-dev \
    libsndfile-dev \
    && rm -rf /var/lib/apt/lists/*

# Compilar SFML v2.5.1 desde código fuente
WORKDIR /opt
RUN git clone --branch 2.5.1 https://github.com/SFML/SFML.git && \
    cd SFML && \
    cmake -DCMAKE_BUILD_TYPE=Release . && \
    make -j$(nproc) && \
    make install

# Copiar y compilar el proyecto
WORKDIR /app
COPY . /app
RUN mkdir -p build
WORKDIR /app/build
RUN cmake .. && make
