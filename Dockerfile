FROM ros:humble

COPY ./docker_app ./ros
COPY ./requirements.txt .


RUN apt-get update
RUN apt-get install -y python3 python3-pip

RUN pip install --no-cache-dir -r requirements.txt

ENV DISPLAY=:0

Workdir ros
RUN colcon build