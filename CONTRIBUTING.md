# Contributing

Thanks for helping improve `flight-log-tools`.

## Pull Requests

Before opening a pull request:

- Keep tools small, inspectable, and easy to run locally.
- Avoid adding third-party dependencies unless they materially improve safety or correctness.
- Do not add real flight logs or files containing private location data.
- Run the test suite:

```bash
python -m unittest discover -s tests
```

- Run a compile check:

```bash
python -m py_compile gps-relocation/relocate_px4_ulog.py gps-relocation/relocate_ardupilot_bin.py tests/test_px4_ulog.py tests/test_ardupilot_dataflash.py
```

## CI Checks

Pull requests run GitHub Actions for:

- Ruff linting
- Ruff format checks
- Unit tests on Ubuntu, Windows, and macOS
- Python compile checks

All required CI checks should pass before a PR is merged.

## AI Reviews

AI review tools may be used to provide advisory feedback on pull requests.
AI comments can be useful for readability, edge cases, and documentation gaps,
but they are not merge gates.

Maintainers should rely on deterministic CI checks and human review for merge
decisions.
