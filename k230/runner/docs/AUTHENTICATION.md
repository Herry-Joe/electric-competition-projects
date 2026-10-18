# GitHub Authentication for Push

This repository currently uses HTTPS for pushing updates.

Because the local environment does not persist Git credentials automatically, you may need to authenticate again when pushing.

## Option A: Use GitHub CLI

```bash
gh auth login
git push origin main
```

## Option B: Use a Personal Access Token (PAT)

Generate a GitHub PAT with `repo` scope, then run:

```bash
git remote set-url origin https://<USERNAME>:<TOKEN>@github.com/Herry-Joe/K230-RUNNER.git
git push origin main
```

After pushing, restore the clean remote URL:

```bash
git remote set-url origin https://github.com/Herry-Joe/K230-RUNNER.git
```

## Notes

- Do not commit tokens into the repository.
- If pushing fails with 403, check that the token is still valid and has `repo` permission.
