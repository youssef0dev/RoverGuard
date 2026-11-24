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
