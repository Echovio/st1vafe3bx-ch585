"""Train reproducible feature-based baselines on synthetic or labelled CSV data."""

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import GroupShuffleSplit

CHANNELS = ("accel_x_mg", "accel_y_mg", "accel_z_mg", "vafe_lsb")
REQUIRED = ("subject_id", "trial_id", "timestamp_us", "label", *CHANNELS)


def demo_rows(seed=42):
    """Produce artificial signals solely to exercise the pipeline."""
    rng = np.random.default_rng(seed)
    rows = []
    for subject in range(8):
        subject_offset = rng.normal(0, 0.1)
        for label in ("movement_a", "movement_b", "rest"):
            for trial in range(2):
                for i in range(400):
                    t = i / 800
                    active = label != "rest"
                    frequency = 2 if label == "movement_a" else 4
                    wave = np.sin(2 * np.pi * frequency * t)
                    values = (subject_offset + (wave if active else 0) + rng.normal(0, .15),
                              (wave * .5 if active else 0) + rng.normal(0, .15),
                              rng.normal(0, .15),
                              rng.normal(0, 100 if not active else 500))
                    rows.append(dict(subject_id=f"synthetic_{subject}",
                                     trial_id=f"{label}_{trial}", timestamp_us=i * 1250,
                                     label=label, **dict(zip(CHANNELS, values))))
    return rows


def read_csv(path):
    with open(path, encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(REQUIRED) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(sorted(missing))}")
        return list(reader)


def windows(rows, window_size=200, step=100, sampling_rate=800):
    if window_size < 2 or step < 1 or sampling_rate <= 0:
        raise ValueError("window-size must be >= 2, step >= 1 and sampling-rate > 0")
    trials = {}
    for row in rows:
        subject, trial, label = (str(row[k]).strip() for k in ("subject_id", "trial_id", "label"))
        if not subject or not trial or not label:
            raise ValueError("subject_id, trial_id and label must not be empty")
        timestamp = float(row["timestamp_us"])
        signal = np.array([float(row[c]) for c in CHANNELS])
        if not np.isfinite(timestamp) or not np.isfinite(signal).all():
            raise ValueError("timestamps and channels must be finite")
        trials.setdefault((subject, trial), []).append((timestamp, label, signal))
    features, labels, groups = [], [], []
    skipped = 0
    expected_dt = 1_000_000 / sampling_rate
    for (subject, trial), samples in sorted(trials.items()):
        samples.sort(key=lambda item: item[0])
        if len({s[1] for s in samples}) != 1:
            raise ValueError(f"trial {subject}/{trial} must have one label")
        times = np.array([s[0] for s in samples])
        if np.any(np.diff(times) <= 0):
            raise ValueError(f"trial {subject}/{trial} has duplicate timestamps")
        data = np.stack([s[2] for s in samples])
        for start in range(0, len(samples) - window_size + 1, step):
            end = start + window_size
            # Drop windows with missing samples or irregular timestamps.
            if np.any(np.abs(np.diff(times[start:end]) - expected_dt) > expected_dt * .2):
                skipped += 1
                continue
            block = data[start:end]
            # Feature order: [mean, std, rms] for each channel.
            feats = np.column_stack((block.mean(axis=0), block.std(axis=0),
                                     np.sqrt(np.mean(block ** 2, axis=0)))).ravel()
            features.append(feats)
            labels.append(samples[0][1])
            groups.append(subject)
    if not features:
        raise ValueError("no valid windows; check trial length and timestamps")
    return np.asarray(features), np.asarray(labels), np.asarray(groups), skipped


def evaluate(rows, seed=42, window_size=200, step=100, sampling_rate=800):
    x, y, groups, skipped = windows(rows, window_size, step, sampling_rate)
    if len(np.unique(groups)) < 2:
        raise ValueError("at least two distinct subjects are required")
    splitter = GroupShuffleSplit(n_splits=1, test_size=.25, random_state=seed)
    train, test = next(splitter.split(x, y, groups))
    classes = np.unique(y)
    if len(classes) < 2 or set(y[train]) != set(classes) or set(y[test]) != set(classes):
        raise ValueError("each split must contain every class; collect more subjects or change seed")
    outputs = {}
    for name, cols in (("sEMG", [9, 10, 11]), ("IMU", list(range(9))),
                       ("sEMG+IMU", list(range(12)))):
        model = RandomForestClassifier(n_estimators=100, random_state=seed, n_jobs=1,
                                       class_weight="balanced")
        model.fit(x[train][:, cols], y[train])
        predicted = model.predict(x[test][:, cols])
        report = classification_report(y[test], predicted, labels=classes,
                                       output_dict=True, zero_division=0)
        outputs[name] = {"accuracy": accuracy_score(y[test], predicted),
                         "macro_f1": report["macro avg"]["f1-score"],
                         "classification_report": report,
                         "confusion_matrix": confusion_matrix(y[test], predicted,
                                                              labels=classes).tolist()}
    return {"seed": seed, "window_size_samples": window_size, "step_samples": step,
            "sampling_rate_hz": sampling_rate, "skipped_irregular_windows": skipped,
            "classes": classes.tolist(), "train_subjects": sorted(set(groups[train])),
            "test_subjects": sorted(set(groups[test])),
            "train_windows": len(train), "test_windows": len(test),
            "train_class_counts": dict(Counter(y[train])),
            "test_class_counts": dict(Counter(y[test])), "models": outputs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", action="store_true", help="use artificial data, not a performance benchmark")
    source.add_argument("--csv", type=Path, help="labelled sample-level CSV")
    parser.add_argument("--out", type=Path, default=Path("outputs/results.json"))
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--sampling-rate", type=float, default=800)
    parser.add_argument("--window-size", type=int, default=200)
    parser.add_argument("--step", type=int, default=100)
    args = parser.parse_args()
    if args.demo and args.sampling_rate != 800:
        parser.error("synthetic demo is generated at 800 Hz")
    rows = demo_rows(args.seed) if args.demo else read_csv(args.csv)
    result = evaluate(rows, args.seed, args.window_size, args.step, args.sampling_rate)
    result["data_kind"] = "synthetic_demo" if args.demo else "user_supplied"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Wrote {args.out} ({result['data_kind']}).")
    if args.demo:
        print("Synthetic scores are not real-world performance.")


if __name__ == "__main__":
    main()
