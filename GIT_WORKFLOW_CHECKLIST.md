# C3PO Git Workflow Checklist

Use this each time you update firmware code.

## 1) Start Of Session

1. Open the C3PO parent folder in VS Code.
2. Open Source Control.
3. Pull latest changes first (Sync Changes or Pull).
4. Confirm branch is main.

## 2) Make Changes

1. Edit files as needed.
2. Build or test the board project you changed.
3. Re-check Source Control to review changed files.

## 3) Stage And Commit

1. Stage only the files you intend to commit.
2. Enter a clear commit message.
3. Commit.

Commit message examples:
- Update torso animation timing
- Fix SBUS channel parsing on slave board
- Refactor audio trigger handling

## 4) Push To GitHub

1. Click Sync Changes (or Push).
2. Confirm push succeeded.
3. Verify commit appears in GitHub repository.

## 5) If Push Is Rejected

1. Pull or Sync to get remote changes.
2. Resolve conflicts in VS Code.
3. Commit the conflict resolution.
4. Push again.

## 6) Optional Terminal Commands

Run these in repo root when needed:

git status
git pull
git add .
git commit -m "Describe what changed"
git push

## 7) Safety Checks Before Big Commits

1. Confirm you are on branch main.
2. Review staged diff before commit.
3. Avoid committing build artifacts (.pio, build outputs, logs).
4. Keep commits focused to one logical change when possible.
