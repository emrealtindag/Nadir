.PHONY: help install check lint format typecheck test test-unit test-integration clean

help:
	@echo "Nadir Development Commands:"
	@echo "  make install           Install dependencies via Poetry"
	@echo "  make check             Run all quality checks (lint, typecheck, test)"
	@echo "  make lint              Run ruff and black code quality checks"
	@echo "  make format            Auto-format code with black and ruff"
	@echo "  make typecheck         Run static type analysis with mypy"
	@echo "  make test              Run all test suites"
	@echo "  make test-unit         Run unit test suite"
	@echo "  make test-integration  Run integration test suite"
	@echo "  make clean             Remove bytecode and cache directories"

install:
	poetry install --with dev,docs

check: lint typecheck test

lint:
	poetry run ruff check .
	poetry run black --check .

format:
	poetry run black .
	poetry run ruff check --fix .

typecheck:
	poetry run mypy src/

test:
	poetry run pytest tests/ -v

test-unit:
	poetry run pytest tests/unit/ -v

test-integration:
	poetry run pytest tests/integration/ -v

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".ruff_cache" -exec rm -rf {} + 2>/dev/null || true
