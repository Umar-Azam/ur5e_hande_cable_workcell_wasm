#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not inside a git repository: $repo_root"
  exit 1
fi

current_branch="$(git rev-parse --abbrev-ref HEAD)"

git checkout -B gh-pages

# GitHub Pages static serving markers.
touch .nojekyll

if ! git diff --quiet -- .nojekyll; then
  git add .nojekyll
  git commit -m "chore(gh-pages): add static site marker"
fi

git checkout "$current_branch"

echo "Prepared local gh-pages branch."
echo "Next steps:"
echo "  git push -u origin gh-pages"
echo "  Enable Pages in repository settings: source branch = gh-pages (root)"
