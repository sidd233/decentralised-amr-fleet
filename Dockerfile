# Item 22 of docs/BUILD_PLAN.md. Two stages: build with the full Rust toolchain, then
# ship only the compiled binary + the map data in a minimal runtime image — matters
# directly for the "edge-hardware-plausible" claim (docs/PS_AND_ARCHITECTURE.md §1.1's
# Pi/Jetson-Nano framing), not just image hygiene: a bloated image undercuts that claim
# right next to the 1 CPU / 512MB cap this build is meant to prove the binary survives.
#
# Alpine/musl, not Debian/glibc: switched after the Debian-based build (rust:1-slim-
# bookworm, ~285MB base layer) hit a real, reproduced network stall on this dev
# machine — confirmed genuine (not a fluke) across two separate attempts, one fully
# stuck and one crawling at ~100KB/s on the same layer, while a plain `curl` throughput
# test on the same connection ran at ~1.27MB/s. Alpine's much smaller base + musl static
# linking pulls less data and starts faster — a real fix here, and also a better fit for
# the edge-hardware-minimal-footprint story than Debian-slim was to begin with, not just
# a workaround.
#
# Resource caps are NOT set in this file — Docker has no build-time directive for
# CPU/memory limits. They're applied at `docker run`/compose time:
#   docker run --cpus=1 --memory=512m sih26123 robot --id 1 --map maps/<file> --start 1,1 --peers 1
# `docker-compose.yml` (item 25) sets the equivalent via `deploy.resources.limits` for
# the full multi-robot stack. docs/TESTING_PLAN.md's Phase 6 gate is this exact
# combination: rebuild inside this Dockerfile, run under the real cap, confirm the
# process doesn't stall or OOM under normal load — confirmed clean (Decision 12).
#
# Item 24 added a third stage: the React/Vite frontend (Decision 11) is built here too,
# so the final image is genuinely self-contained — `dashboard/server.rs` serves
# `frontend/dist` as static files from inside the container, not from a host bind mount,
# matching Decision 11's "single binary + a few dashboard build files" deployment story.

FROM node:22-alpine AS frontend-builder
WORKDIR /frontend
COPY frontend/package.json frontend/package-lock.json ./
RUN npm ci
COPY frontend/ ./
RUN npm run build

FROM rust:1-alpine AS builder
WORKDIR /build

# musl-dev + build-base: several crates in the dependency tree (e.g. anything with a
# build.rs invoking a C compiler) assume one is present even when not linking against a
# specific C library — cheap to install, avoids an opaque build.rs failure later.
RUN apk add --no-cache musl-dev build-base

# Cargo.toml/Cargo.lock copied first so dependency compilation is cached across rebuilds
# that only change src/ — Cargo.lock is committed and never hand-edited (docs/FILE_MAP.md),
# so this is a reproducible build, not a moving target.
COPY Cargo.toml Cargo.lock ./
COPY src ./src
RUN cargo build --release --bin sih26123

# Runtime stage: plain alpine, not scratch/distroless — still has a real shell (`ash`)
# and `apk`, which this project's smoke tests (docs/decisions.md Decision 10) already
# found real debugging value in (`docker exec` into a running container), just far
# smaller than Debian-slim.
FROM alpine:3.20
WORKDIR /app

COPY --from=builder /build/target/release/sih26123 /usr/local/bin/sih26123
COPY --from=frontend-builder /frontend/dist ./frontend/dist
COPY maps ./maps

ENTRYPOINT ["/usr/local/bin/sih26123"]
