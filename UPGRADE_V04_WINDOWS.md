# Upgrade an existing RoverGuard checkout to v0.4 on Windows

Run commands from the existing repository root.

```powershell
cd C:\Users\hp\robot-lab\roverguard
git switch -c feature/multi-point-world
```

Download `RoverGuard-v0.4-Multi-Point.zip`, then locate it:

```powershell
$zip = Get-ChildItem "$env:USERPROFILE\Downloads","$env:USERPROFILE\Desktop" -Filter "RoverGuard-v0.4-Multi-Point.zip" -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
$zip
```

Extract it outside the repository and copy its contents over the repository.
The archive does not contain `.git`, so Git history remains intact.

```powershell
$source = "$env:USERPROFILE\Desktop\RoverGuard-v0.4-Multi-Point"
Expand-Archive -Path $zip -DestinationPath $source -Force
Test-Path "$source\run_cognitive_demo.py"
Copy-Item -Path "$source\*" -Destination "C:\Users\hp\robot-lab\roverguard" -Recurse -Force
```

Expected `Test-Path` result: `True`.

Run tests:

```powershell
& "..\.venv\Scripts\python.exe" -m unittest discover -s tests -v
```

Expected: `Ran 22 tests` and `OK`.

Start Ollama if needed:

```powershell
Start-Process -FilePath "ollama.exe" -ArgumentList "serve"
```

Run the external mission console:

```powershell
& "..\.venv\Scripts\python.exe" .\run_cognitive_demo.py
```

Example command to type when prompted:

```text
Sir P2, P5, P10, w rje3 l BASE b speed 0.15 m/s
```
