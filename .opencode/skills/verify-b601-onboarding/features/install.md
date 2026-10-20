# Install and discover

## Sub-features

Install the release wheel, check dependency consistency, discover plugins, and read CLI help.

## How to get to it (user POV)

Follow the README installation block on a Linux computer with Python 3.10 through 3.12.

## Driving it with the Python CLI

Use the Launch, Doctor, and Drive commands in `../SKILL.md`. Check that registry output contains `b601_ros` and `b601_motorbridge`. Read the package-test output and confirm eight tests passed. Verify the wheel has no removed realtime extra or console script through the existing installed-package test.

## Gotchas

The package is not published to PyPI. Use the exact GitHub wheel URL. Installing or listing plugins does not connect to a robot. A newly built wheel and the published release are different artifacts and need separate environments.
