FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# deps
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY . .

# build
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON \
 && cmake --build build -j"$(nproc)"

# unit tests
RUN cd build && ctest --output-on-failure 

FROM ubuntu:22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        can-utils \
        iproute2 \
        libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=builder /build/build/cansimulator .

ENTRYPOINT ["./cansimulator"]
CMD ["--help"]
