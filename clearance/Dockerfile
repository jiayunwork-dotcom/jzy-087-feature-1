# syntax=docker/dockerfile:1

# --- build stage -----------------------------------------------------------
FROM golang:1.22-bookworm AS build

WORKDIR /src

# Resolve modules first so dependency downloads are cached independently of
# source changes.
COPY go.mod go.sum ./
RUN go mod download

COPY . .

# Pure-Go static binary (no libc dependency), with VCS info stripped.
ENV CGO_ENABLED=0 GOOS=linux
RUN go build -trimpath -ldflags="-s -w" -o /out/clearance-svc ./cmd/server

# --- runtime stage ---------------------------------------------------------
# base image ships busybox (used by HEALTHCHECK); the binary itself is a
# fully static pure-Go executable and needs no libc.
FROM gcr.io/distroless/base-debian12:nonroot

WORKDIR /
COPY --from=build /out/clearance-svc /usr/local/bin/clearance-svc

EXPOSE 8080
USER nonroot:nonroot

HEALTHCHECK --interval=30s --timeout=3s --retries=3 \
  CMD ["/bin/busybox", "wget", "-qO-", "http://127.0.0.1:8080/health"]

ENTRYPOINT ["/usr/local/bin/clearance-svc"]
