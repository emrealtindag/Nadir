# Contributing to Nadir

Thank you for your interest in contributing to **Nadir**! We welcome contributions from the community to help make autonomous precision landing safer, faster, and more robust.

---

## Code of Conduct

Please maintain a respectful, constructive, and inclusive tone across all interactions, issues, and pull requests.

---

## Development Workflow

1. **Fork and Clone**:
   ```bash
   git clone https://github.com/emrealtindag/nadir.git
   cd nadir
   ```

2. **Environment Setup**:
   Install dependencies using [Poetry](https://python-poetry.org/):
   ```bash
   poetry install --with dev,docs
   ```

3. **Branching Strategy**:
   Create a descriptive branch for your feature or bugfix:
   ```bash
   git checkout -b feature/my-feature-name
   # or
   git checkout -b fix/issue-description
   ```

4. **Code Quality Standards**:
   Nadir enforces strict coding standards:
   - **Formatting**: `black .`
   - **Linting**: `ruff check .`
   - **Type Checking**: `mypy src/`
   - **Tests**: `pytest tests/`

   You can verify everything with:
   ```bash
   poetry run ruff check .
   poetry run black --check .
   poetry run pytest tests/ -v
   ```
   Or using `make`:
   ```bash
   make check
   ```

5. **Commit Conventions**:
   Write clear, concise commit messages following standard conventions:
   - `feat: add adaptive search pattern for low altitude`
   - `fix: handle edge case in PnP coplanar marker rejection`
   - `docs: update Gazebo Harmonic SITL launch instructions`

6. **Submitting a Pull Request**:
   - Push your branch to your fork.
   - Open a Pull Request targeting `main`.
   - Provide a clear summary of what your changes achieve, along with any simulation logs or SITL validation results.

---

## License

By contributing to Nadir, you agree that your contributions will be licensed under the project's [Apache License 2.0](LICENSE).
