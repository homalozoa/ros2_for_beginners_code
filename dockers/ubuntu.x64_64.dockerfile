# Copyright (c) 2022 Homalozoa
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
ARG FROM_IMAGE=amd64/ubuntu:bionic
ARG gfw=0
FROM $FROM_IMAGE AS base-image
ENV CMAKE_VER="3.21.2"

FROM base-image AS build-in-gfw-1
ENV GITHUB_URL="hub.fastgit.org"
ENV GITHUB_RAW="raw.fastgit.org"

FROM base-image AS build-in-gfw-0
ENV GITHUB_URL="github.com"
ENV GITHUB_RAW="raw.githubusercontent.com"

FROM build-in-gfw-${gfw} AS final

# replace mirror (replace one near you)
RUN apt-get update && \
  apt-get install -q -y --no-install-recommends ca-certificates && \
  mv /etc/apt/sources.list /etc/apt/sources.list.bak && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse\n" > /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse\n" >> /etc/apt/sources.list && \
  echo "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse" >> /etc/apt/sources.list

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && \
  apt-get install -q -y --no-install-recommends tzdata

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  bash-completion \
  dirmngr \
  gnupg2 \
  lsb-release \
  python3-pip

# setup ros2 sources.list (replace one near you)
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ bionic main" > /etc/apt/sources.list.d/ros2-latest.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  git \
  wget \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool

# install python packages
RUN pip3 install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest \
  pytest-repeat \
  pytest-rerunfailures \
  conan \
  lark-parser && \
  python3 -m pip install -U importlib-metadata importlib-resources numpy

# remove cache
RUN apt-get clean

# setup entrypoint
RUN echo '#!/bin/bash\n' > /ros_entrypoint.sh && \
  echo 'set -e\n' >> /ros_entrypoint.sh && \
  echo '# setup ros2 environment\n' >> /ros_entrypoint.sh && \
  echo 'source /opt/ros/galactic/setup.sh\n' >> /ros_entrypoint.sh && \
  echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n' >> /ros_entrypoint.sh && \
  echo 'exec "$@"' >> /ros_entrypoint.sh && \
  chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
