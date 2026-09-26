# Contributing

How we work on this repo together. If you have not used git much, read it top to
bottom once — it is written to be followed literally, and the
[When it goes wrong](#when-it-goes-wrong) section at the end is there for when
it does.

**The one rule:** nobody commits to `main` directly. You work on a branch, open
a pull request, Adam reviews it, and then it merges. GitHub enforces this, so
you cannot get it wrong by accident.

---

## One-time setup

You need [Docker](https://docs.docker.com/get-started/get-docker/) and git.

```bash
git clone https://github.com/Adam-Axelrod/unibots-redemption.git
cd unibots-redemption
```

Tell git who you are, if you never have:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

Check the simulator runs before you change anything, so that if something breaks
later you know it was you and not the setup:

```bash
docker compose up
```

Open <http://localhost:8080>. You should see a room with a robot in it.

Optional but recommended — [GitHub's CLI](https://cli.github.com/), which lets
you open a pull request without leaving the terminal:

```bash
brew install gh     # or: https://cli.github.com
gh auth login
```

---

## The everyday loop

Five commands, in this order, every time. The rest of this section is just
explaining them.

```bash
git switch main && git pull      # 1. start from everyone else's latest
git switch -c my-feature-name    # 2. make your own branch
# ... edit code, test it in the sim ...
git add -A && git commit -m "Add a wall-following driver"   # 3. save your work
git push -u origin my-feature-name                          # 4. send it to GitHub
gh pr create                                                # 5. ask for it to be merged
```

### 1. Start from the latest `main`

```bash
git switch main
git pull
```

Do this **every time you start something new**. If you branch off a stale `main`
you will be merging against changes from a week ago, and that is where painful
conflicts come from.

### 2. Make a branch

```bash
git switch -c wall-follower
```

A branch is a private copy of the project where your changes cannot disturb
anyone else. Name it after what you are doing — `wall-follower`,
`fix-lidar-crash`, `faster-sweep`. Lowercase with dashes.

Check which branch you are on any time with `git status` (the first line) or
`git branch --show-current`.

### 3. Commit as you go

A commit is a save point. Make one whenever something works, not just at the end.

```bash
git add -A
git commit -m "Stop the robot when the lidar sees a wall"
```

Write the message as an instruction: *"Add the wall follower"*, not *"added wall
follower"* or *"stuff"*. If you cannot describe it in one line, that is usually
a sign it should be two commits.

Small and often beats one enormous commit. Nobody has ever regretted committing
too frequently.

### 4. Push to GitHub

```bash
git push -u origin wall-follower
```

The `-u origin wall-follower` part is only needed the **first** push of a new
branch. After that, `git push` on its own is enough.

Pushing does not merge anything or affect anyone. It just puts your branch on
GitHub so it is backed up and other people can see it. Push often.

### 5. Open a pull request

A pull request ("PR") says: *here is my finished work, please put it in `main`.*

```bash
gh pr create --fill --web
```

Or go to the repo on github.com — there will be a **Compare & pull request**
button at the top.

Fill in the template: what it does, and how you tested it. "Tested in
`furnished_room`, the robot completes a lap without hitting anything" tells a
reviewer far more than "works".

**Not finished but want feedback?** Open it as a draft:

```bash
gh pr create --fill --draft
```

That is encouraged. A draft PR early beats a surprise PR late.

### 6. Review, then merge

Every PR needs **Adam's approval** before the **Merge** button turns green.
Anyone can comment on anyone's PR, and reading each other's is the fastest way
to learn what the rest of the robot does.

If the review asks for changes, you do not open a new PR. Just commit and push
to the same branch — the PR updates itself:

```bash
# ... make the changes ...
git add -A && git commit -m "Use a parameter for the stopping distance"
git push
```

When it is approved, click **Squash and merge**. Your branch is deleted
automatically.

### 7. Clean up and go again

```bash
git switch main
git pull
git branch -d wall-follower      # delete your finished local branch
```

---

## What CI checks

When you open a PR, GitHub automatically builds the workspace and runs every
node against the simulator. It takes a few minutes. A red ✗ means one of:

| Check | What it means |
|---|---|
| **Every node registered in setup.py** | You added a node but did not add it to `console_scripts`. The error message tells you the exact line to paste. |
| **colcon build** | Python syntax error, or an import that does not resolve. |
| **Smoke test** | Your node crashed within 8 seconds, ignored Ctrl-C, or the robot did not move. |

Click **Details** next to the failed check to see the log. Fix it, commit, push,
and the check runs again by itself.

You can run the first check on your own machine in a second, before you push:

```bash
python3 .github/scripts/check_entrypoints.py
```

---

## What a good change looks like here

These are the house rules from [CLAUDE.md](CLAUDE.md). The reviewer will look
for them, and the PR template lists them as a checklist.

- **Register new nodes in `setup.py`** under `console_scripts`, or `ros2 run`
  will not find them. This catches everyone once.
- **Publish `/cmd_vel` on a timer, continuously.** Both the simulator and the
  real firmware stop the robot if commands go quiet — that is a safety feature.
  Publishing once and expecting the robot to keep going will not work.
- **Publish a zero `Twist` on shutdown, and catch `ExternalShutdownException`
  as well as `KeyboardInterrupt`.** On Ctrl-C rclpy shuts the ROS context down
  before `spin()` returns, so a node that catches only `KeyboardInterrupt`
  exits with a traceback. Copy the whole `main()` from `square_driver.py` and
  you get this right for free. CI fails a node that does not stop cleanly.
- **Parameters, not constants.** Use `declare_parameter` so numbers can be tuned
  with `--ros-args -p speed:=0.3` instead of by editing code.
- **Python only.** No C++ packages.
- **Never commit `ws/build/`, `ws/install/` or `ws/log/`.** They are generated,
  and they are already in `.gitignore`.
- **Do not branch on "am I in simulation".** The whole design is that the sim
  and the robot publish identical topics. If you need that branch, something
  else has gone wrong — say so in the PR and we will work it out.

Copy [`square_driver.py`](ws/src/robot_control/robot_control/square_driver.py)
as your starting point. It is about 80 lines and shows the entire shape of a
control node.

---

## When it goes wrong

Git problems all look alarming and are almost all reversible. Find yours.

### "I made changes but I am on `main`"

You forgot to branch. Nothing is lost — move the changes onto a branch:

```bash
git switch -c my-feature-name
```

Your uncommitted changes come with you. Carry on as normal.

### "I already committed to `main`"

Move the commit to a branch and put `main` back:

```bash
git switch -c my-feature-name    # branch, with your commit on it
git switch main
git reset --hard origin/main     # main back to matching GitHub
git switch my-feature-name       # back to your work
```

> `git reset --hard` throws away uncommitted changes in the current branch.
> Here that is what we want, because the work is safely on the new branch. Be
> careful with it elsewhere.

### "My push was rejected"

```
! [rejected] main -> main (fetch first)
```

Someone pushed while you were working. Get their changes, then push again:

```bash
git pull --rebase
git push
```

If you are on `main` and it says `protected branch`, that is the rule working
correctly — branch first, see above.

### "I have a merge conflict"

Two people changed the same lines. Git marks them in the file:

```
<<<<<<< HEAD
        self.speed = 0.25
=======
        self.speed = 0.4
>>>>>>> main
```

Open the file, delete the `<<<<<<<`, `=======` and `>>>>>>>` lines, and leave
the code you want (often a combination of both). Then:

```bash
git add -A
git rebase --continue    # or `git commit` if you were merging
```

Lost halfway through and want out? `git rebase --abort` puts everything back the
way it was.

### "I committed `ws/build/` by accident"

```bash
git rm -r --cached ws/build ws/install ws/log
git commit -m "Remove generated build output"
```

`--cached` removes them from git but leaves the files on your disk.

### "I want to undo my last commit"

Keep the changes, undo the commit:

```bash
git reset --soft HEAD~1
```

Throw the changes away too (cannot be undone):

```bash
git reset --hard HEAD~1
```

### "I have no idea what state I am in"

```bash
git status                  # what has changed, what branch
git log --oneline -10       # the last 10 commits
git branch --show-current   # just the branch name
```

Paste the output into the group chat. Somebody will recognise it — and if a
command looks destructive and you are not sure, **ask before running it**.
Anything already pushed to GitHub can be recovered.

<!-- protection check, this branch is deleted straight after -->
