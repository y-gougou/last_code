#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Build run-level train/test fault-diagnosis windows from raw CSV logs."""

from __future__ import print_function

import argparse
import csv
import glob
import json
import os
import pickle
from collections import Counter, defaultdict

import numpy as np
import pandas as pd


DEFAULT_FEATURES = [
    "cmd_vx",
    "cmd_vy",
    "cmd_wz",
    "odom_vx",
    "odom_vy",
    "odom_wz",
    "imu_ax",
    "imu_ay",
    "imu_az",
    "imu_gx",
    "imu_gy",
    "imu_gz",
    "voltage",
    "current0",
    "current1",
    "current2",
]

WHEEL_SPEED_FEATURES = ["wheel_speed0", "wheel_speed1", "wheel_speed2"]
LABEL_NAMES = {
    0: "normal",
    1: "drive_fault",
    2: "wheel_slip",
    3: "shaft_eccentric",
    4: "encoder_fault",
}


class SimpleScaler(object):
    def __init__(self, method):
        self.method = method
        self.center_ = None
        self.scale_ = None
        self.data_min_ = None
        self.data_max_ = None

    def fit(self, x):
        if self.method == "standard":
            self.center_ = x.mean(axis=0)
            self.scale_ = x.std(axis=0)
            self.scale_[self.scale_ < 1e-12] = 1.0
        elif self.method == "minmax":
            self.data_min_ = x.min(axis=0)
            self.data_max_ = x.max(axis=0)
            self.scale_ = self.data_max_ - self.data_min_
            self.scale_[self.scale_ < 1e-12] = 1.0
        else:
            raise ValueError("unsupported scaler: %s" % self.method)
        return self

    def transform(self, x):
        if self.method == "standard":
            return (x - self.center_) / self.scale_
        return (x - self.data_min_) / self.scale_


def load_csvs(input_dir, pattern):
    paths = sorted(glob.glob(os.path.join(input_dir, pattern)))
    if not paths:
        raise RuntimeError("no CSV files found: %s/%s" % (input_dir, pattern))

    frames = []
    for path in paths:
        df = pd.read_csv(path)
        if "run_id" not in df.columns:
            df["run_id"] = os.path.splitext(os.path.basename(path))[0]
        if "fault_name" not in df.columns and "fault_label" in df.columns:
            df["fault_name"] = df["fault_label"].map(
                lambda v: LABEL_NAMES.get(int(v), "unknown")
            )
        df["source_file"] = os.path.basename(path)
        frames.append(df)
    return pd.concat(frames, ignore_index=True), paths


def choose_features(df, include_wheel_speed):
    features = list(DEFAULT_FEATURES)
    if include_wheel_speed == "auto":
        use_wheel = all(col in df.columns for col in WHEEL_SPEED_FEATURES)
        if use_wheel:
            total_abs = df[WHEEL_SPEED_FEATURES].abs().sum().sum()
            use_wheel = bool(total_abs > 1e-9)
    else:
        use_wheel = include_wheel_speed == "yes"
    if use_wheel:
        features.extend(WHEEL_SPEED_FEATURES)

    missing = [col for col in features if col not in df.columns]
    if missing:
        raise RuntimeError("missing model feature columns: %s" % ", ".join(missing))
    return features


def clean_dataframe(df, features):
    required = list(features) + ["fault_label", "run_id"]
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise RuntimeError("missing required columns: %s" % ", ".join(missing))

    df = df.copy()
    for col in features + ["fault_label"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.replace([np.inf, -np.inf], np.nan)

    bad_rows = df[features + ["fault_label"]].isna().any(axis=1).sum()
    if bad_rows:
        print("dropping rows with NaN/Inf: %d" % bad_rows)
        df = df.dropna(subset=features + ["fault_label"])

    df["fault_label"] = df["fault_label"].astype(int)
    illegal = sorted(set(df["fault_label"]) - set(LABEL_NAMES))
    if illegal:
        raise RuntimeError("illegal labels found: %s" % illegal)
    return df


def split_runs(df, test_ratio, explicit_test_runs):
    runs = sorted(df["run_id"].astype(str).unique())
    if explicit_test_runs:
        test_runs = sorted(set(explicit_test_runs))
    else:
        by_label = defaultdict(list)
        run_labels = (
            df.groupby("run_id")["fault_label"]
            .agg(lambda s: int(s.mode().iloc[0]))
            .to_dict()
        )
        for run_id in runs:
            by_label[run_labels[run_id]].append(run_id)

        test_runs = []
        for _, label_runs in sorted(by_label.items()):
            n_test = max(1, int(round(len(label_runs) * test_ratio)))
            test_runs.extend(label_runs[-n_test:])
        test_runs = sorted(set(test_runs))

    train_runs = [run for run in runs if run not in test_runs]
    if not train_runs or not test_runs:
        raise RuntimeError("train/test run split is empty; collect more runs or pass --test_runs")
    return train_runs, test_runs


def fit_transform_by_train(df, features, train_runs, scaler_method):
    train_mask = df["run_id"].astype(str).isin(train_runs)
    scaler = SimpleScaler(scaler_method).fit(df.loc[train_mask, features].values.astype(np.float32))
    scaled = df.copy()
    scaled[features] = scaler.transform(df[features].values.astype(np.float32))
    return scaled, scaler


def window_one_run(run_df, features, window_size, step_size):
    run_df = run_df.sort_values("sample_id") if "sample_id" in run_df.columns else run_df
    x_values = run_df[features].values.astype(np.float32)
    labels = run_df["fault_label"].values.astype(np.int64)
    rows = []
    metas = []

    if len(run_df) < window_size:
        return rows, metas

    for start in range(0, len(run_df) - window_size + 1, step_size):
        end = start + window_size
        window_labels = labels[start:end]
        label = Counter(window_labels.tolist()).most_common(1)[0][0]
        window = x_values[start:end].T
        rows.append((window, label))

        first = run_df.iloc[start]
        last = run_df.iloc[end - 1]
        metas.append(
            {
                "run_id": first["run_id"],
                "source_file": first.get("source_file", ""),
                "start_row": int(start),
                "end_row": int(end - 1),
                "fault_label": int(label),
                "fault_name": first.get("fault_name", LABEL_NAMES.get(int(label), "unknown")),
                "motion_mode": first.get("motion_mode", ""),
                "start_time": first.get("timestamp", ""),
                "end_time": last.get("timestamp", ""),
            }
        )
    return rows, metas


def make_windows(df, features, runs, window_size, step_size):
    windows = []
    labels = []
    metas = []
    per_run = {}

    for run_id in runs:
        run_df = df[df["run_id"].astype(str) == str(run_id)]
        run_windows, run_metas = window_one_run(run_df, features, window_size, step_size)
        per_run[run_id] = len(run_windows)
        for window, label in run_windows:
            windows.append(window)
            labels.append(label)
        metas.extend(run_metas)

    if not windows:
        raise RuntimeError("no windows created for runs: %s" % ", ".join(runs))
    return np.stack(windows), np.asarray(labels, dtype=np.int64), metas, per_run


def write_meta(path, metas):
    if not metas:
        return
    with open(path, "w") as f:
        writer = csv.DictWriter(f, fieldnames=list(metas[0].keys()))
        writer.writeheader()
        writer.writerows(metas)


def validate_arrays(name, x, y):
    if not np.isfinite(x).all():
        raise RuntimeError("%s contains NaN or Inf" % name)
    if not np.isfinite(y).all():
        raise RuntimeError("%s labels contain NaN or Inf" % name)
    print("%s: X=%s y=%s labels=%s" % (name, x.shape, y.shape, dict(Counter(y.tolist()))))


def save_outputs(output_dir, result):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    np.save(os.path.join(output_dir, "X_train.npy"), result["X_train"])
    np.save(os.path.join(output_dir, "y_train.npy"), result["y_train"])
    np.save(os.path.join(output_dir, "X_test.npy"), result["X_test"])
    np.save(os.path.join(output_dir, "y_test.npy"), result["y_test"])
    write_meta(os.path.join(output_dir, "meta_train.csv"), result["meta_train"])
    write_meta(os.path.join(output_dir, "meta_test.csv"), result["meta_test"])

    with open(os.path.join(output_dir, "scaler.pkl"), "wb") as f:
        pickle.dump(result["scaler"], f, protocol=2)

    with open(os.path.join(output_dir, "feature_config.json"), "w") as f:
        json.dump(result["feature_config"], f, indent=2, sort_keys=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--pattern", default="*.csv")
    parser.add_argument("--window_size", type=int, default=50)
    parser.add_argument("--step_size", type=int, default=10)
    parser.add_argument("--scaler", choices=["standard", "minmax"], default="standard")
    parser.add_argument("--test_ratio", type=float, default=0.2)
    parser.add_argument("--test_runs", default="")
    parser.add_argument("--include_wheel_speed", choices=["auto", "yes", "no"], default="auto")
    args = parser.parse_args()

    df, paths = load_csvs(args.input_dir, args.pattern)
    features = choose_features(df, args.include_wheel_speed)
    df = clean_dataframe(df, features)

    explicit_test_runs = [x.strip() for x in args.test_runs.split(",") if x.strip()]
    train_runs, test_runs = split_runs(df, args.test_ratio, explicit_test_runs)
    scaled, scaler = fit_transform_by_train(df, features, train_runs, args.scaler)

    X_train, y_train, meta_train, train_counts = make_windows(
        scaled, features, train_runs, args.window_size, args.step_size
    )
    X_test, y_test, meta_test, test_counts = make_windows(
        scaled, features, test_runs, args.window_size, args.step_size
    )

    validate_arrays("train", X_train, y_train)
    validate_arrays("test", X_test, y_test)
    print("train runs:", train_runs)
    print("test runs:", test_runs)
    print("windows per train run:", train_counts)
    print("windows per test run:", test_counts)

    feature_config = {
        "features": features,
        "feature_groups": {
            "cmd_odom": [c for c in features if c.startswith("cmd_") or c.startswith("odom_")],
            "imu": [c for c in features if c.startswith("imu_")],
            "electrical": [c for c in features if c == "voltage" or c.startswith("current")],
            "wheel_speed": [c for c in features if c.startswith("wheel_speed")],
        },
        "input_shape": ["num_samples", len(features), args.window_size],
        "window_size": args.window_size,
        "step_size": args.step_size,
        "scaler": args.scaler,
        "label_names": LABEL_NAMES,
        "source_files": [os.path.basename(p) for p in paths],
        "train_runs": train_runs,
        "test_runs": test_runs,
    }

    save_outputs(
        args.output_dir,
        {
            "X_train": X_train,
            "y_train": y_train,
            "X_test": X_test,
            "y_test": y_test,
            "meta_train": meta_train,
            "meta_test": meta_test,
            "scaler": scaler,
            "feature_config": feature_config,
        },
    )
    print("saved dataset to:", args.output_dir)


if __name__ == "__main__":
    main()
