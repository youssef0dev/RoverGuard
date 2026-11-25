# Cognitive mission examples

```text
Inspect P1 and P3, then return to base.
```

```text
Sir l P2, dir inspection, w rje3 l BASE b speed 0.15 m/s.
```

```text
Visite P3 puis P1 et retourne a la base lentement.
```

Plan-only test:

```powershell
python run_cognitive_demo.py `
  --command "Inspect P1 and P3, then return to base" `
  --plan-only
```

Multi-point warehouse examples:

```text
Sir P2, P5, P10, w rje3 l BASE b speed 0.15 m/s.
```

```text
Inspect P9, P4 and P6, then return to base slowly.
```

```text
Visite P8 puis P3 et P5, ensuite retourne a BASE.
```

Photo-action example:

```text
Go to P3 and take a photo, then go to P6, then go to P8 and take a photo, then return to BASE.
```

Expected action split:

```text
waypoints: P3, P6, P8, BASE
photo_waypoints: P3, P8
```
