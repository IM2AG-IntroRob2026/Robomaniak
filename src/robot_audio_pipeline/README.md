# robot_audio_pipeline

Pipeline audio ROS 2 (etape 1):

1. Capture micro via PortAudio
2. Inference wake-word via Snowboy
3. Publication d'evenements de detection
4. Execution de commandes robot simples (via hotwords)

## Architecture

- `audio_pipeline_node`:
  - lit l'audio depuis le peripherique d'entree (`audio_input_device_index`),
  - envoie les frames PCM a Snowboy,
  - publie le mot-cle detecte sur `detections_topic`,
  - publie un niveau RMS audio sur `rms_topic` (optionnel).

- `voice_command_node`:
  - s'abonne a `detections_topic`,
  - mappe les hotwords vers des commandes `geometry_msgs/Twist`,
  - publie sur `cmd_vel_topic`.
  - mapping par defaut:
    - `avance` -> avance pendant `3.0s`
    - `gauche` -> rotation gauche de `45 deg`
    - `droite` -> rotation droite de `45 deg`

- `PortAudioInput`:
  - abstraction capture audio bloquante frame-par-frame.

- `SnowboyEngine`:
  - abstraction initialisation + process Snowboy.

## Topics publies

- `std_msgs/String` sur `/audio/detected_hotword`
- `std_msgs/Float32` sur `/audio/rms` (si active)
- `geometry_msgs/Twist` sur `/audio/cmd_vel` (par defaut via `voice_command_node`)

## Parametres principaux

- `resource_path` (obligatoire): chemin du `common.res`
- `hotword_model_paths` (obligatoire): liste des `.pmdl/.umdl`
- `keyword_labels` (optionnel): labels de sortie associes
- `hotword_sensitivities` (optionnel): une valeur globale ou une par hotword
- `audio_input_device_index` (default `-1`)
- `detection_cooldown_ms` (anti-repetition)
- `forward_keyword` / `left_keyword` / `right_keyword`
- `forward_duration_s` / `turn_angle_deg`
- `cmd_vel_topic`

## Lancement

```bash
ros2 launch robot_audio_pipeline audio_pipeline.launch.py \
  resource_path:=/path/to/common.res
```

Configurer les `hotword_model_paths`/`keyword_labels` dans:

- `config/audio_pipeline.yaml`

## Remarque

Les labels detectes par Snowboy doivent correspondre aux mots attendus dans
`voice_command_node` (`avance`, `gauche`, `droite` par defaut).
