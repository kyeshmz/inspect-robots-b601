# pip installation and publishing

The project is a normal PEP 517/621 Python distribution, with versioned wheels,
source archives, entry points and `py.typed` markers. Installing a wheel or a
public Git tag does not require PyPI publication:

```bash
python -m pip install 'git+https://github.com/kyeshmz/inspect-robots-b601.git@v0.4.0'
```

Use Python 3.10–3.12. Wheels are also attached to
[GitHub Releases](https://github.com/kyeshmz/inspect-robots-b601/releases).
The optional `agent` and `motorbridge` dependencies remain opt-in. The package
does not require a hosted service or remote transport.

## Enable the short PyPI name once

`pip install inspect-robots-b601` searches PyPI. It will not work until the
project has actually been published there. A repository or a green build badge
does not publish it automatically.

The prepared `.github/workflows/publish.yml` uses
[PyPI Trusted Publishing](https://docs.pypi.org/trusted-publishers/), not a
long-lived API token. In your PyPI account, register a **pending publisher** for
a new project at <https://pypi.org/manage/account/publishing/>:

| Field | Value |
| --- | --- |
| PyPI project name | `inspect-robots-b601` |
| GitHub owner | `kyeshmz` |
| Repository | `inspect-robots-b601` |
| Workflow filename | `publish.yml` |
| Environment | `pypi` |

Use a GitHub `pypi` environment with appropriate maintainer approval rules.
Only the publishing job receives `id-token: write`. The workflow checks a real
version tag, runs quality/tests, builds distributions and validates the installed
wheel before asking PyPI for a short-lived upload credential.

After that account-side setup, a maintainer can explicitly dispatch:

```bash
gh workflow run publish.yml --repo kyeshmz/inspect-robots-b601 -f tag=v0.4.0
```

Do not dispatch before the pending publisher is configured. The workflow does
not create an account, claim a project name, guess credentials, change an
existing tag or silently skip an upload failure. Never paste PyPI tokens or
model API keys into chat, a workflow file or a package artifact.

After a successful upload, verify the PyPI project and a fresh install before
advertising the short form:

```bash
python -m pip install 'inspect-robots-b601==0.4.0'
```

Published versions are immutable; choose a new version for a changed artifact.
