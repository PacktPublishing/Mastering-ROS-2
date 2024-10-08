#Base Image
FROM osrf/ros:jazzy-desktop-full

#Update and upgrade Ubuntu package and install pip
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip

#Execute command, bash when it create the container
CMD ["/bin/bash"]

