# A from-scratch Ubuntu 20.04 (glibc 2.31) image for the `cross` cargo
# subcommand, used instead of the stock ghcr.io/cross-rs/aarch64-unknown-linux-gnu
# image (Ubuntu 24.04 / glibc up to 2.39). The XGO-Lite V2's stock Raspberry
# Pi OS image only ships glibc <= 2.31, so a robot-edge binary built against
# the stock cross-rs image fails to even start on the robot ("version
# GLIBC_2.39 not found"). Building fresh from ubuntu:20.04 (rather than
# layering focal packages on top of the noble-based cross-rs image) avoids
# apt resolving the newer noble cross-libc packages instead of focal's.
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --assume-yes --no-install-recommends \
        gcc-aarch64-linux-gnu \
        g++-aarch64-linux-gnu \
        cmake git build-essential ca-certificates curl pkg-config \
    && rm -rf /var/lib/apt/lists/*

# flatc must be v1.11.0 to match the `flatbuffers = "0.7"` crate pinned in
# crates/roboprotocol-proto/Cargo.toml (see .github/workflows/rust.yml) --
# a newer flatc generates Rust bindings (Verifier/InvalidFlatbuffer-based)
# that this old flatbuffers crate version doesn't have.
RUN git clone --depth 1 --branch v1.11.0 https://github.com/google/flatbuffers.git /tmp/flatbuffers-src \
    && sed -i -E 's/-Werror=/-W/g; s/-Werror\b//g' /tmp/flatbuffers-src/CMakeLists.txt \
    && cmake -S /tmp/flatbuffers-src -B /tmp/flatbuffers-build -DCMAKE_BUILD_TYPE=Release -DFLATBUFFERS_BUILD_TESTS=OFF \
    && cmake --build /tmp/flatbuffers-build --target flatc -j"$(nproc)" \
    && install -m 755 /tmp/flatbuffers-build/flatc /usr/local/bin/flatc \
    && rm -rf /tmp/flatbuffers-src /tmp/flatbuffers-build

# Tell `cross`/cargo/cc-rs/cmake-rs which cross compiler to use for the
# aarch64-unknown-linux-gnu target -- same convention cross-rs's own images use.
ENV CROSS_TOOLCHAIN_PREFIX=aarch64-linux-gnu-
ENV CROSS_SYSROOT=/usr/aarch64-linux-gnu
ENV CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER="${CROSS_TOOLCHAIN_PREFIX}gcc" \
    AR_aarch64_unknown_linux_gnu="${CROSS_TOOLCHAIN_PREFIX}ar" \
    CC_aarch64_unknown_linux_gnu="${CROSS_TOOLCHAIN_PREFIX}gcc" \
    CXX_aarch64_unknown_linux_gnu="${CROSS_TOOLCHAIN_PREFIX}g++" \
    BINDGEN_EXTRA_CLANG_ARGS_aarch64_unknown_linux_gnu="--sysroot=${CROSS_SYSROOT}"
