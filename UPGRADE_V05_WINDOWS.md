# Upgrade RoverGuard v0.4 to v0.5 on Windows

From the existing repository:

```powershell
cd C:\Users\hp\robot-lab\roverguard
git status --short
git switch -c feature/mission-camera-actions
```

Locate and extract the downloaded archive:

```powershell
$zip = Get-ChildItem "$env:USERPROFILE\Downloads","$env:USERPROFILE\Desktop" -Filter "RoverGuard-v0.5-Camera-Actions.zip" -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
$source = "$env:USERPROFILE\Desktop\RoverGuard-v0.5-Camera-Actions"
Expand-Archive -Path $zip -DestinationPath $source -Force
Test-Path "$source\roverguard\camera.py"
Copy-Item -Path "$source\*" -Destination "C:\Users\hp\robot-lab\roverguard" -Recurse -Force
```

Install the new image dependency and run tests:

```powershell
& "..\.venv\Scripts\python.exe" -m pip install -r .\requirements.txt
& "..\.venv\Scripts\python.exe" -m unittest discover -s tests -v
```

Expected: `Ran 25 tests` and `OK`.

Run the photo mission console:

```powershell
& "..\.venv\Scripts\python.exe" .\run_cognitive_demo.py
```

Enter:

```text
Go to P3 and take a photo, then go to P6, then go to P8 and take a photo, then return to BASE.
```

List captures after mission success:

```powershell
Get-ChildItem .\captures\*.png | Sort-Object LastWriteTime
```
