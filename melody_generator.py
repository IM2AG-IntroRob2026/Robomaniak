import math

# Fréquences des notes (octave 4 comme référence)
NOTE_BASE = {
    "DO": 261.63,
    "DO#": 277.18, "REb": 277.18,
    "RE": 293.66,
    "RE#": 311.13, "MIb": 311.13,
    "MI": 329.63,
    "FA": 349.23,
    "FA#": 369.99, "SOLb": 369.99,
    "SOL": 392.00,
    "SOL#": 415.30, "LAb": 415.30,
    "LA": 440.00,
    "LA#": 466.16, "SIb": 466.16,
    "SI": 493.88
}


def note_frequency(tone, octave=4):
    """Calcule la fréquence selon l'octave."""
    base = NOTE_BASE[tone.upper()]
    return round(base * (2 ** (octave - 4)))


def duration_ns(time, tempo):
    """
    Convertit une durée musicale en nanosecondes.
    time = fraction de noire
    ex: 1 = noire, 0.5 = croche
    """
    beat_duration = 60 / tempo
    return int(beat_duration * time * 1e9)


def generate_sequence(sequence):
    tempo = sequence["tempo"]
    output = []

    for note in sequence["notes"]:
        tone = note["tone"]
        octave = note.get("octave", 4)
        time = note["time"]

        freq = note_frequency(tone, octave)
        ns = duration_ns(time, tempo)

        output.append(
            {"frequency": freq, "max_runtime": {"sec": 0, "nanosec": ns}}
        )

        # petite pause entre les notes
        output.append(
            {"frequency": 0, "max_runtime": {"sec": 0, "nanosec": 1}}
        )

    return output


def print_sequence(seq):
    print("notes: [")
    for n in seq:
        print(f'{{frequency: {n["frequency"]}, max_runtime: {{sec: {n["max_runtime"]["sec"]}, nanosec: {n["max_runtime"]["nanosec"]}}}}},')
    print("]")


in_the_end = {
    "tempo": 106,
    "notes": [
        {"tone": "RE", "time": 0.25},
        {"tone": "LA", "time": 1},
        {"tone": "LA", "time": 1},
        {"tone": "FA", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 0.5},
        {"tone": "FA", "time": 0.5},
        {"tone": "RE", "time": 0.25},
        {"tone": "LA", "time": 1},
        {"tone": "LA", "time": 1},
        {"tone": "FA", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 0.5},
        {"tone": "FA", "time": 0.5},
        {"tone": "RE", "time": 0.25},
        {"tone": "LA", "time": 1},
        {"tone": "LA", "time": 1},
        {"tone": "FA", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 0.5},
        {"tone": "FA", "time": 0.5},
        {"tone": "RE", "time": 0.25},
        {"tone": "LA", "time": 1},
        {"tone": "LA", "time": 1},
        {"tone": "FA", "time": 1},
        {"tone": "MI", "time": 1},
        {"tone": "MI", "time": 2},
    ]
}

seq = generate_sequence(in_the_end)
print_sequence(seq)
