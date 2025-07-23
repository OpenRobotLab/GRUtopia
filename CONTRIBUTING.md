# Contributing to InternUtopia

Thank you for your interest in contributing to InternUtopia! We welcome contributions from everyone. Please take a moment to review this guide to ensure a smooth collaboration.

- [Reporting Bugs](https://github.com/InternRobotics/InternUtopia/issues/new/choose)
- [Suggesting Enhancement](https://github.com/InternRobotics/InternUtopia/issues/new/choose)
- [Questions and Discussions](https://github.com/InternRobotics/InternUtopia/issues/new/choose)
- [Submitting Code Changes](#submitting-code-changes)
- [Reviewing and Merging](#reviewing-and-merging)

## Submitting Code Changes

- We use the `pre-commit` configuration to automatically clean up code before committing. Install and run `pre-commit` as follows:
  1. Install `pre-commit`:

    ```bash
    pip install pre-commit
    ```

  2. Install hooks from the configuration file at the root of the repository:

    ```bash
    pre-commit install
    ```

    After this, `pre-commit` will automatically check and clean up code whenever you make a commit.

  3. Before opening a pull request, run the following command to check all files are cleaned up:

    ```bash
    pre-commit run --all-files
    ```

- In the title of your Pull Request, please include [BUG FIX], [FEATURE] or [MISC] to indicate the purpose.
- In the description, please provide example code or commands for testing.
- For commit messages, please follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification.

## Reviewing and Merging

- PRs require at least one approval before merging.
- Automated checks (e.g., CI tests) must pass.
- Use `Squash and Merge` for a clean commit history.
