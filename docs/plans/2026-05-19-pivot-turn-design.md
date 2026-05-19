# Pivot Turn — Design

**Date:** 2026-05-19
**Fichier:** `esibot_bringup/esibot_bringup/esibot_driver.py`

## Problème

Quand l'opérateur appuie sur `j` ou `l` dans `teleop_twist_keyboard` (rotation pure, sans avancer), les **deux roues tournent en sens opposés** (spin-in-place). Le comportement attendu est un **pivot turn** : une roue tourne, l'autre reste à l'arrêt.

| Touche | Actuel | Attendu |
|--------|--------|---------|
| `l` (angular > 0) | droite avant + gauche arrière | droite avant, gauche stop |
| `j` (angular < 0) | gauche avant + droite arrière | gauche avant, droite stop |

## Cause

La cinématique dans `_cmd_vel_callback` applique toujours la formule différentielle standard :

```python
v_right = linear + (angular * WHEEL_BASE / 2.0)
v_left  = linear - (angular * WHEEL_BASE / 2.0)
```

Quand `linear == 0`, cela produit des vitesses opposées sur les deux roues.

## Solution — Pivot turn conditionnel

Modifier uniquement `_cmd_vel_callback`. Quand `linear == 0` et `angular ≠ 0`, appliquer la formule pivot :

```python
if linear == 0.0 and angular != 0.0:
    v_right = max(0.0,  angular) * WHEEL_BASE
    v_left  = max(0.0, -angular) * WHEEL_BASE
else:
    v_right = linear + (angular * WHEEL_BASE / 2.0)
    v_left  = linear - (angular * WHEEL_BASE / 2.0)
```

## Comportement résultant

| Touche | angular | v_right | v_left |
|--------|---------|---------|--------|
| `l` | > 0 | avant | stop |
| `j` | < 0 | stop | avant |
| `i` | 0 | égal | égal |
| `u`,`o`,`m`,`.` | ≠ 0 + linear ≠ 0 | standard | standard |

## Impact

- Aucun effet sur l'odométrie, SLAM, Nav2.
- Le contrôle moteur reste binaire (full speed / stop) — la valeur numérique dépasse toujours `MOTOR_DEADBAND`.
- Les touches diagonales (`u`, `o`, `m`, `.`) conservent la cinématique différentielle standard.
