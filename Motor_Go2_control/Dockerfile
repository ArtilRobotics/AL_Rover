# Ubuntu Base
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Instala dependencias necesarias
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libserialport-dev \
    libspnav-dev \
    libx11-dev \
    libsfml-audio-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Crea el directorio de trabajo
WORKDIR /app

# Copia los archivos al contenedor
COPY . /app

# Crea carpeta build
RUN mkdir build
WORKDIR /app/build

# Compila y construye el proyecto
RUN cmake ..
RUN make
