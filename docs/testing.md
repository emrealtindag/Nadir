# Testing Guide

Nadir maintains automated test suites covering both unit components and full system data integration.

---

## Test Execution

### Running Unit Tests
```bash
poetry run pytest tests/unit/ -v
```

### Running Integration Tests
```bash
poetry run pytest tests/integration/ -v
```

### Running All Tests
```bash
poetry run pytest tests/ -v
```

### Code Formatting & Static Analysis
```bash
poetry run ruff check .
poetry run black --check .
poetry run mypy src/
```

Or execute everything via `make`:
```bash
make check
```
