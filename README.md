# ArmForge — 6-DOF Robotic Arm Digital Twin

Interactive digital twin of a 6-axis industrial arm (UR5-style), built as a layered **C++23 / Qt 6** portfolio project demonstrating real-time robotics software architecture.

> **Status:** Phases 0–5 complete and running. Phase 6 (jumeau physique) est une piste d'évolution future.

---

## 🗺️ Roadmap

| Phase | Livrable | État |
|-------|----------|------|
| **0 — Spike** | Scène Qt Quick 3D, bras statique en primitives, Eigen branché, build OK | ✅ Done |
| **1 — FK** | Sliders → angles → FK C++ → hiérarchie 3D animée en temps réel | ✅ Done |
| **2 — IK** | Gizmo 3D draggable → solveur DLS → l'effecteur suit la cible | ✅ Done |
| **3 — Contrôle** | Boucle 50 Hz sur thread dédié, interpolation LERP, oscilloscope live | ✅ Done |
| **4 — Planification** | Waypoints, évitement d'obstacle, planificateur RRT (CPU) | ✅ Done |
| **5 — CUDA** | `CudaRrtPlanner` derrière `IMotionPlanner`, benchmark CPU/GPU dans l'UI | ✅ Done |
| **6 — Jumeau physique** *(idée)* | Import CAO réelle, liaison MCU (capteurs/actionneurs), carte d'obstacles dynamique | 💡 Piste |

---

## ✅ Ce qui est implémenté (Phase 0–4)

### Scène & rendu (Qt Quick 3D)
- `View3D` avec caméra perspective orbitale, 4 lumières directionnelles, sol avec ombres MSAA
- Hiérarchie de `Node` miroir de la chaîne cinématique : J1 (Y) → J2 (Z) → J3 (Z) → J4 (X) → J5 (Y) → J6 (X)
- Matériaux `PrincipledMaterial` (métal/rugosité) sur chaque segment

### Cinématique directe — Phase 1
- `RobotController` singleton QML expose 6 propriétés d'angle (degrés)
- 6 sliders dans `JogPanel` → `setAngle()` → `recomputeFK()` → la scène se met à jour en temps réel
- Tous les angles de joint sont calculés via une FK visuelle (matrices 4×4 Eigen) qui correspond exactement à la hiérarchie QML

### Cinématique inverse — Phase 2
- Solveur **Damped Least Squares** : `Δq = Jᵀ(JJᵀ + λ²I)⁻¹·err` — robuste aux singularités
- Jacobienne numérique calculée colonne par colonne (perturbation `ε = 1e-3 rad`)
- **FK visuelle** (non-DH) : chaîne `Trans(0,119,0)·Ry(q₀) · Rz(q₁) · Trans(425,0,0)·Rz(q₂) · Trans(392,0,0)·Rx(q₃) · Trans(0,0,109)·Ry(q₄) · Trans(0,0,95)·Rx(q₅)` — axes identiques au QML
- **Gizmo 3D** (sphère + axes X/Y/Z) positionné sur la cible IK :
  - Clic gauche + drag → déplacement XZ par intersection rayon-plan
  - Clic droit + drag → déplacement Y avec échelle profondeur-dépendante
- Couleur du gizmo : vert (CONVERGED) / rouge (UNREACHABLE)
- Affichage position cible en mètres dans `JogPanel`

### Boucle de contrôle — Phase 3
- `ControlLoop` (QObject) tourne sur un **QThread dédié** à **50 Hz** (timer 20 ms)
- Interpolation linéaire par axe : `q(t) = q_start + α·(q_target − q_start)` avec `α = min(t/T, 1)`
- Communication cross-thread via `Qt::QueuedConnection` (zéro mutex sur le chemin chaud)
- **Durée de trajectoire configurable** : slider 0.0 → 3.0 s dans `JogPanel`
  - `0 s` → snap instantané (1 tick = 20 ms)
  - `0.5 s` → suivi réactif du gizmo (défaut)
  - `2–3 s` → visualisation dramatique de la trajectoire
- **Oscilloscope** (`ChartPanel`) : 6 courbes colorées sur les 300 derniers échantillons (~6 s), mis à jour à 50 Hz en mode IK, ring-buffer JS

### Planification de mouvement — Phase 4
- **`IMotionPlanner`** : interface abstraite (`AF_Core`, zéro Qt) — point d'extension pour le `CudaRrtPlanner` de la Phase 5
- **`RrtConnectPlanner`** (CPU, pur Eigen) : RRT-Connect bidirectionnel dans l'espace articulaire
  - primitives `extend`/`connect`, arbres start/goal qui échangent leur rôle à chaque itération
  - `steer()` borné par `stepSize` (0.12 rad), validation de segment discrétisée (`collisionResolution` 0.05 rad)
  - obstacles sphériques (`SphereObstacle`) + `isClearOfObstacles()` avec marge de sécurité configurable
- **`PlanningController`** (singleton QML, orchestration) :
  - **TEACH** : capture la config articulaire courante comme waypoint (avec sa position EE)
  - **PLAN PATH** : enchaîne les segments courant → wp[0] → wp[1] → … via RRT-Connect, concatène en une seule trajectoire, expose `pathPoints` (trace EE) et `pathConfigsDeg` (configs pour rejouer)
  - vérification de collision basée sur la "FK visuelle" partagée (`RobotController::visualFrames`) — mêmes points que la scène rendue
- **`RobotController::followPath()`** : rejoue la trajectoire planifiée à travers la `ControlLoop` existante à 50 Hz (même machinerie LERP que le mode IK), piloté par un `QTimer` par segment
- **`PlanningPanel`** : liste de waypoints, boutons TEACH / CLEAR / PLAN PATH / RUN PATH, statut (succès/itérations)
- **`PlanningView`** (`Repeater3D`, Qt Quick 3D) : sphères d'obstacles (rouge translucide), marqueurs de waypoints (ambre), trace de l'effecteur planifiée (points verts)

### Accélération GPU — Phase 5
- **`collision_kernel.cu/.cuh`** : kernel CUDA `k_batchValidate` — un thread par configuration articulaire, boucle sur ses points d'échantillonnage × les obstacles sphériques, sortie anticipée à la première collision. C'est la contrepartie parallèle massive de la boucle CPU séquentielle
- **`CudaRrtPlanner`** : miroir quasi exact de `RrtConnectPlanner` (même stratégie RRT-Connect bidirectionnelle), seule `segmentValidGpu()` change — elle calcule la FK de tout un segment discrétisé puis valide le lot entier en **un seul lancement de kernel** au lieu de N appels CPU séquentiels
- **Benchmark CPU vs GPU** dans `PlanningPanel` : génère un lot aléatoire de configurations, chronomètre la boucle CPU (`isClearOfObstacles`) puis l'unique appel `af::cuda::batchValidate`, affiche les deux temps et le facteur d'accélération
- **Toggle "PLANIFIER SUR GPU"** : bascule `plan()` entre `RrtConnectPlanner` et `CudaRrtPlanner` (les deux implémentent `IMotionPlanner` — le reste du flux est identique)
- **CMake** : `ARMFORGE_ENABLE_CUDA` (détection via `find_package(CUDAToolkit)`), fallback CPU transparent si le toolkit/driver est absent ou trop ancien. `collision_kernel.cu` est compilé via un `add_custom_command` qui invoque `nvcc` directement (arguments cités avec `VERBATIM`), plutôt que via les règles natives `enable_language(CUDA)` — celles-ci mal-citent la ligne de commande sous le générateur "NMake Makefiles JOM" de ce kit Qt Creator

---

## 💡 Pistes d'évolution (au-delà de Phase 5)

Trois idées qui me trottent dans la tête pour la suite, histoire de faire passer ArmForge d'un jumeau purement numérique à quelque chose qui dialogue avec un vrai bras robotique.

### 1. Importer la géométrie 3D réelle du bras
C'est sans doute la piste la plus accessible des trois. Qt Quick 3D sait charger des modèles `glTF2` nativement, donc un export CAO (SolidWorks ou Fusion 360, en passant par STEP/IGES puis une conversion via FreeCAD ou Blender) pourrait remplacer directement les cylindres et sphères primitifs de `RobotView.qml`. Le plus gros du travail serait sur le pipeline d'assets plutôt que sur le code : la hiérarchie de `Node` qui suit la chaîne cinématique resterait la même, il faudrait juste recaler les `DhParameters` sur les vraies dimensions. Et tant qu'à avoir une géométrie réelle sous la main, ce serait l'occasion d'affiner le modèle de collision — passer des points d'échantillonnage actuels à des capsules collées à chaque lien.

### 2. Brancher le bras à un microcontrôleur STM32 (et un MPU)
Là ça devient plus ambitieux, et ça déborde clairement du C++/Qt actuel — ce serait plutôt un sous-projet firmware à mener à côté. Deux sens de communication envisageables, qu'on pourrait combiner :
- du capteur vers le twin : un STM32 lit un MPU6050/9250 en I2C et envoie les angles en UART/USB, et côté PC un `QSerialPort` (le module `Qt6::SerialPort` s'ajoute facilement au `CMakeLists.txt`) récupère le flux et alimente `RobotController` — le jumeau refléterait alors la pose réelle du bras
- du twin vers les actionneurs : à l'inverse, on sérialise `pathConfigsDeg` et on l'envoie au STM32 pour qu'il pilote les vrais servomoteurs — le twin deviendrait la source de vérité du mouvement

La partie Qt (lire/écrire sur le port série) reste assez directe à écrire. La vraie difficulté est côté firmware — driver I2C, boucle de contrôle moteur — et surtout dans la conception d'un protocole série robuste (trames à fréquence fixe, gestion des déconnexions, etc.).

### 3. Planifier avec une carte d'obstacles réelle plutôt qu'un décor figé
Pour l'instant `m_obstacles` est une liste de deux sphères codées en dur. Deux façons très différentes d'aborder ça :
- la voie pragmatique : quelques capteurs de distance (ultrasons ou ToF) à des positions connues, dont les mesures arriveraient par le même pont série pour peupler `m_obstacles` dynamiquement (il suffirait de rendre la propriété QML notifiable au lieu de `CONSTANT`). L'intérêt, c'est que tout le reste — `isClearOfObstacles`, les deux planificateurs, la vue 3D — fonctionne déjà avec des sphères : rien à changer côté algorithmes, juste la source des données change
- la voie "cartographie temps réel" — caméra de profondeur, SLAM, reconstruction de nuages de points — qui est un projet de recherche à part entière, et clairement hors de portée raisonnable pour ArmForge tel qu'il est pensé aujourd'hui

---

## 🧱 Architecture en couches

```
┌─────────────────────────────────────────────────────────────┐
│  QML UI  (Qt Quick 3D + Qt Quick Controls 2)                │
│  SceneView · RobotView · GizmoView · JogPanel · ChartPanel  │
├─────────────────────────────────────────────────────────────┤
│  Controllers (QML_ELEMENT / QML_SINGLETON)                  │
│  RobotController — FK, IK, contrôle, propriétés QML        │
├─────────────────────────────────────────────────────────────┤
│  app/control — Boucle temps réel                           │
│  ControlLoop (QThread, 50 Hz) — LERP, stateUpdated signal  │
├─────────────────────────────────────────────────────────────┤
│  AF_Core — Domaine pur (Eigen, ZÉRO Qt)                    │
│  ForwardKinematics · IKSolver (DLS) · KinematicChain       │
└─────────────────────────────────────────────────────────────┘
```

**Règle de dépendance** : les flèches ne pointent que vers le bas. `AF_Core` ne dépend que d'Eigen — compilable et testable sans Qt.

---

## ⚙️ Stack technique

| Domaine | Choix |
|---------|-------|
| **UI / 3D** | Qt 6.11 · Qt Quick 3D · Qt Quick Controls 2 |
| **Langage** | C++23 (MSVC 2022) |
| **Algèbre linéaire** | Eigen 3.4 (matrices 4×4 homogènes, Jacobienne, LDLT) |
| **Build** | CMake ≥ 3.21 + vcpkg (Eigen via manifest) |
| **Threading** | QThread + QueuedConnection (cross-thread signal/slot) |
| **Rendu** | Qt Quick 3D, primitives intégrées, PrincipledMaterial, MSAA |

---

## 🔧 Build

```bash
# Avec vcpkg (Eigen via manifest)
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release

# Sans vcpkg (FetchContent fallback automatique)
cmake -B build -S .
cmake --build build --config Release
```

**Prérequis :** Qt 6.6+, CMake ≥ 3.21, compilateur C++23 (MSVC 2022 / GCC 12+ / Clang 15+), Eigen 3.4

### Build rapide sous Windows (sans ouvrir Qt Creator)

Le script [`build.ps1`](build.ps1) reproduit exactement la configuration du kit Qt Creator
"Desktop Qt 6.11.0 MSVC2022 64bit" (lecture de `.qtcreator/CMakeLists.txt.user` et
`CMakeCache.txt`) : il charge l'environnement de développement MSVC (`vcvars64.bat`),
ajoute `jom` au PATH et invoque le CMake embarqué de Qt avec le générateur
`NMake Makefiles JOM`.

```powershell
.\build.ps1               # build incrémental (cas le plus courant)
.\build.ps1 -Reconfigure  # relance la configuration CMake (après modif de CMakeLists.txt)
.\build.ps1 -Clean        # build de la cible "clean"
.\build.ps1 -Run          # build puis lance ArmForge.exe
```

---

**Auteur** : Abdelmounem Azaiez
