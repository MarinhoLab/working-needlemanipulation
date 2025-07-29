FROM ubuntu:24.04
LABEL authors="Murilo Marinho"

ENTRYPOINT ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install sudo git python3-pip cmake libeigen3-dev -y