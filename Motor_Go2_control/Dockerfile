# Usa Ubuntu como sistema base
FROM ubuntu:22.04

# Evita preguntas interactivas durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# Instala dependencias necesarias
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libserialport-dev \
    libspnav-dev \
    libx11-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Crea el directorio de trabajo
WORKDIR /app

# Copia los archivos al contenedor
COPY . /app

# Crea carpeta build
RUN mkdir build
WORKDIR /app/build

# Construye el proyecto con cmake y make
RUN cmake ..
RUN make
