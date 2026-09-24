# Scandium Docker Image
# Production-grade container for precision landing system
#
# Build: docker build -f docker/Dockerfile -t scandium:latest .
# Run: docker run -it --rm scandium:latest scandium version

# ------------------------------------------------------------------------------
# Build Stage
# ------------------------------------------------------------------------------
FROM python:3.11-slim-bookworm AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Poetry
ENV POETRY_HOME=/opt/poetry
ENV PATH="${POETRY_HOME}/bin:${PATH}"
RUN pip install poetry==1.8.0

# Set working directory
WORKDIR /app

# Copy dependency files
COPY pyproject.toml poetry.lock* ./

# Configure Poetry for container environment
ENV POETRY_NO_INTERACTION=1 \
    POETRY_VIRTUALENVS_IN_PROJECT=1 \
    POETRY_VIRTUALENVS_CREATE=1

# Install dependencies only (not the package itself)
RUN poetry install --without dev,docs --no-root

# Copy source code
COPY src/ ./src/
COPY configs/ ./configs/
COPY README.md ./

# Install the package
RUN poetry install --without dev,docs

# ------------------------------------------------------------------------------
# Runtime Stage
# ------------------------------------------------------------------------------
FROM python:3.11-slim-bookworm AS runtime

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-core4.6 \
    libopencv-imgproc4.6 \
    libopencv-calib3d4.6 \
    libopencv-objdetect4.6 \
    libopencv-imgcodecs4.6 \
    libopencv-videoio4.6 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create non-root user for security
RUN groupadd --gid 1000 scandium \
    && useradd --uid 1000 --gid 1000 --shell /bin/bash --create-home scandium

# Set working directory
WORKDIR /app

# Copy virtual environment from builder
COPY --from=builder /app/.venv /app/.venv
ENV PATH="/app/.venv/bin:${PATH}"

# Copy application files
COPY --from=builder /app/src /app/src
COPY --from=builder /app/configs /app/configs
COPY docker/entrypoint.sh /app/entrypoint.sh

# Set permissions
RUN chown -R scandium:scandium /app \
    && chmod +x /app/entrypoint.sh

# Create output directory
RUN mkdir -p /app/runs && chown scandium:scandium /app/runs

# Switch to non-root user
USER scandium

# Environment variables
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    SCANDIUM_LOG_FORMAT=json

# Expose port for MAVLink UDP (if needed for debugging)
EXPOSE 14550/udp

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD scandium diagnostics --config /app/configs/default.yaml || exit 1

# Default entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["scandium", "version"]
