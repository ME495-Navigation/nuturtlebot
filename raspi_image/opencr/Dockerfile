# This dockerfile is for building the opencr firmware
FROM ubuntu:jammy

RUN dpkg --add-architecture i386 && \
    apt-get update && \
    apt-get install -y \
    curl \
    libc6:i386 \
    libc++6:i386 \
    git

# Clone (my fork of) the opencr repository
RUN git clone https://github.com/ME495-Navigation/OpenCR -b humble_stock

# Install the arduino CLI from instructions https://arduino.github.io/arduino-cli/0.29/installation/
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install the OpenCR board files
COPY arduino-cli.yaml /arduino-cli.yaml

RUN ln -s /OpenCR/arduino/opencr_release /arduino && \
    arduino-cli --config-file /arduino-cli.yaml core install OpenCR:OpenCR -v && \
    arduino-cli --config-file /arduino-cli.yaml core update-index && \
    arduino-cli --config-file /arduino-cli.yaml lib update-index 

