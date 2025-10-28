
FROM ubuntu:22.04 AS base

RUN apt update && apt install -y \
	clang \
	cmake \
	python3 python3-pip

RUN apt update && apt install -y \
	libboost-serialization-dev \
	libboost-filesystem-dev \
	libboost-system-dev \
	libboost-program-options-dev \
	libboost-test-dev

COPY . /app

WORKDIR /app/build

RUN clang --version

CMD ["sh", "-c", "cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 all && ./main"]
