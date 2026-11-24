# Upgrade an existing v0.2 Git checkout on Windows

1. Confirm the current branch:

```powershell
git branch --show-current
```

Expected:

```text
feature/cognitive-mission-layer
```

2. Extract this v0.3 archive outside the existing repository.

3. Copy all extracted files over the existing repository root. Keep the
existing hidden `.git` directory; this archive does not contain one.

4. Run the tests:

```powershell
& "..\.venv\Scripts\python.exe" -m unittest discover -s tests -v
```

Expected: `Ran 19 tests` and `OK`.

5. Test only the local-LLM plan:

```powershell
& "..\.venv\Scripts\python.exe" .\run_cognitive_demo.py `
  --command "Inspect P1 and P3, then return to base" `
  --plan-only
```

6. Start the approved mission in PyBullet:

```powershell
& "..\.venv\Scripts\python.exe" .\run_cognitive_demo.py `
  --command "Inspect P1 and P3, then return to base"
```

7. After both tests pass:

```powershell
git add .
git commit -m "feat: add safe local-LLM cognitive mission layer"
```
